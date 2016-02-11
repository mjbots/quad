// Copyright 2014-2016 Mikhail Afanasyev.  All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "camera_driver.h"

#include <mutex>
#include <thread>

#include <gst/gst.h>

#include <boost/format.hpp>

#include "base/common.h"
#include "base/context_full.h"
#include "base/fail.h"
#include "base/json_archive.h"
#include "base/logging.h"
#include "base/now.h"

#include "gst_helpers.h"

namespace mjmech {
namespace mech {

namespace {

struct Preset {
  double framerate;
  double decoded_framerate;
  double bitrate_mbps;
  const char* h264_caps;
  const char* decoded_caps;
  const char* extra_uvch264;
};

// To get list of supported resolutions, use: v4l2-ctl --list-formats-ext
const Preset kPresets[] = {{
 framerate         : 30,
 decoded_framerate : 0,
 bitrate_mbps      : 3.0,
 h264_caps         : "width=1920,height=1080",
 decoded_caps      : "width=640,height=480",
 extra_uvch264     : "",
  }, {
 framerate         : 30, // Actually 15-24 fps
 decoded_framerate : 0,
 bitrate_mbps      : 1.5,
 h264_caps         : "width=1280,height=720",
 decoded_caps      : "width=640,height=480",
 extra_uvch264     : "",
  }, {
 framerate         : 30,  // Actually 15 fps
 decoded_framerate : 0,
 bitrate_mbps      : 0.75,
 h264_caps         : "width=864,height=480",
 decoded_caps      : "width=640,height=480",
 extra_uvch264     : "",
 // TODO theamk: figure out if we can decrease the framerate somehow (currenly,
 // values other than 30 just prevent piepline from starting)
}};

};

class CameraDriver::Impl : boost::noncopyable {
 public:
  Impl(CameraDriver* parent, boost::asio::io_service& service)
      : parent_(parent),
        parent_service_(service),
        parent_id_(std::this_thread::get_id()),
        stats_(new CameraStats()) {}

  ~Impl() {
    if (gst_loop_) {
      gst_loop_->WaitForQuit();
    }
  }

  void AsyncStart(base::ErrorHandler handler) {
    BOOST_ASSERT(std::this_thread::get_id() == parent_id_);
    BOOST_ASSERT(!gst_loop_);

    started_ = true;
    parameters_ = parent_->parameters_;

    // Apply presets
    const int kMaxPresets = sizeof(kPresets) / sizeof(kPresets[0]) - 1;

    int i_preset = parameters_.preset;
    base::ErrorCode error;

    if (i_preset < 0 || i_preset > kMaxPresets) {
      std::string msg = (boost::format("Preset %d is not in the range 0..%d")
                         % i_preset % kMaxPresets).str();
      parent_service_.post(std::bind(handler, base::ErrorCode::einval(msg)));
      return;
    }
    const Preset& preset = kPresets[i_preset];

    if (parameters_.framerate < 0) {
      parameters_.framerate = preset.framerate;
    }
    if (parameters_.decoded_framerate < 0) {
      parameters_.decoded_framerate = preset.decoded_framerate;
    }
    if (parameters_.bitrate_mbps < 0) {
      parameters_.bitrate_mbps = preset.bitrate_mbps;
    }
    if (!parameters_.h264_caps.size()) {
      parameters_.h264_caps = preset.h264_caps;
    }
    if (!parameters_.decoded_caps.size()) {
      parameters_.decoded_caps = preset.decoded_caps;
    }
    if (!parameters_.extra_uvch264.size()) {
      parameters_.extra_uvch264 = preset.extra_uvch264;
    }

    parent_service_.post(std::bind(handler, base::ErrorCode()));
  }

  void AddFrameConsumer(std::weak_ptr<CameraFrameConsumer> c) {
    BOOST_ASSERT(std::this_thread::get_id() == parent_id_);
    BOOST_ASSERT(!started_);
    consumers_.push_back(c);
  }

  void HandleGstReady(GstMainLoopRef& loop_ref) {
    gst_loop_ = loop_ref;
    if (!started_) {
      log_.debug("component disabled");
      return;
    }
    SetupPipeline();
  }

 private:
  std::string MakeLaunchCmd() {
    std::ostringstream out;

    std::string device = parameters_.device;
    if (device == "")  {
      device = "/dev/video0";
    }

    bool is_test = device == "TEST";
    bool is_dumb = parameters_.dumb_camera || is_test;
    dumb_camera_ = is_dumb;

    int bitrate_bps = static_cast<int>(
        ::round(parameters_.bitrate_mbps * parameters_.bitrate_scale
                * 1000 * 1000));

    if (is_test) {
      out << "videotestsrc name=testsrc is-live=1 pattern=ball ";
    } else if (is_dumb) {
      out << "v4l2src name=src device=" << gst::PipelineEscape(device)
          << " ! videoconvert ";
    } else {
      out << "uvch264src device=" << gst::PipelineEscape(device)
          << " name=src auto-start=true message-forward=true";
      if (parameters_.iframe_interval_s > 0) {
        out << " iframe-period=" <<
            static_cast<int>(parameters_.iframe_interval_s * 1000.0);
      }
      if (bitrate_bps > 0) {
        int bitrate_peak_bps = static_cast<int>(
            ::round(bitrate_bps * parameters_.peak_bitrate_scale));
        out << " average-bitrate=" << bitrate_bps
            << " initial-bitrate=" << bitrate_bps
            << " peak-bitrate=" << bitrate_peak_bps;
      }
      out << " " << parameters_.extra_uvch264
          << " src.vfsrc ";
    }

    // Make decoded image endpoint
    double decoded_fps =
        (parameters_.decoded_framerate > 0)
        ? parameters_.decoded_framerate
        : parameters_.framerate;
    out << "! video/x-raw,format=I420,framerate="
        << gst::FormatFraction(decoded_fps) << ","
        << parameters_.decoded_caps << " ";

    if (is_dumb) {
      out << "! tee name=dec-tee ";
    };
    if (parameters_.analyze) {
      out << " ! videoanalyse name=input-analyze ";
    }
    out << "! queue ! appsink name=raw-sink ";


    // Make H264 endpoint
    if (is_dumb) {
      out << "dec-tee. ! videoconvert ! queue "
          << " ! x264enc tune=zerolatency byte-stream=true ";
      if (parameters_.iframe_interval_s) {
        int key_int = 0.5 + decoded_fps * parameters_.iframe_interval_s;
        out << "key-int-max=" << std::max(1, key_int) << " ";
      }
      if (bitrate_bps > 0) {
        out << "bitrate=" << (bitrate_bps / 1000) << " ";
      }
    } else {
      out << "src.vidsrc";
    }
    out << "! video/x-h264,framerate="
        << gst::FormatFraction(parameters_.framerate);
    if (!is_dumb) {
      // If this is not a dumb camera, force a h264 resolution (if it is a
      // dumb camera, the h264 resolution is forced to be the the same as
      // input resultion)
      out << "," << parameters_.h264_caps;
    };
    out << " ! h264parse ";

    if (parameters_.write_video != "" ||
        parameters_.custom_h264_consumer != "") {
      out << "! tee name=h264-tee ";
    }
    // Maybe save it
    if (parameters_.write_video != "") {
      std::string muxer = "mp4mux";
      std::string tail = parameters_.write_video.substr(
          std::max(0, static_cast<int>(parameters_.write_video.size()) - 4));
      if (tail == ".mkv") {
        muxer = "matroskamux streamable=true";
      } else if (tail == ".avi") {
        muxer = "avimux";
      } else if (tail != ".mp4") {
        log_.error("Unknown h264 savefile extension " + tail +
                   ", assuming MP4 format");
      }
      // h264parse is this magic component which will change the h264 stream
      // format as required (for example, mp4 wants stream-format=avc)
      out << "! queue ! h264parse ! " << muxer
          << " ! filesink name=h264writer location="
          << gst::PipelineEscape(parameters_.write_video)
          << " h264-tee. ";
    }
    // Maybe serve it to custom consumer
    if (parameters_.custom_h264_consumer != "") {
      out << "! queue ! " << parameters_.custom_h264_consumer
          << " h264-tee. ";
    }
    // And pass it to our app (with the right caps)
    out << "! queue ! appsink name=h264-sink "
        << " caps=video/x-h264,stream-format=byte-stream,alignment=au ";
    return out.str();
  }

  void SetupPipeline() {
    std::string launch_cmd = MakeLaunchCmd();

    pipeline_.reset(
        new gst::PipelineWrapper(
            gst_loop_, "camera_driver", launch_cmd));

    // Hook the deep nitification
    pipeline_->ConnectElementSignal(
         NULL, "deep-notify",
         G_CALLBACK(handle_deep_notify_wrapper), this);

    pipeline_->SetupAppsink(
        "raw-sink", 1, true,
        std::bind(&Impl::HandleRawSinkNewSample, this,
                  std::placeholders::_1));

    pipeline_->SetupAppsink(
        "h264-sink", 120, true,
        std::bind(&Impl::HandleH264SinkNewSample, this,
                  std::placeholders::_1));

    pipeline_->RegisterVideoAnalyzeMessageHandler(
        "input-analyze",
        std::bind(&Impl::HandleVideoAnalyzeMessage,
                  this, std::placeholders::_1));

    if (parameters_.device == "TEST") {
      video_test_src_ = pipeline_->GetElementByName("testsrc");
    }

    pipeline_->Start();

    // Set up the timer for stats
    if (parameters_.stats_interval_s > 0) {
      gst_loop_->AddPeriodicTimer(
           parameters_.stats_interval_s,
           std::bind(&Impl::HandleStatsTimeout, this));
    }
  }

  void HandleVideoAnalyzeMessage(const gst::VideoAnalyzeMessage& msg) {
    log_.infoStream() << "raw frame info: "
                      << msg.toString();
  }


  static gboolean handle_deep_notify_wrapper(
      GstObject* gstobject, GstObject* prop_object,
      GParamSpec* prop, gpointer user_data) {
    static_cast<CameraDriver::Impl*>(user_data)
        ->HandleDeepNotify(gstobject, prop_object, prop);
    return TRUE;
  }

  // WARNING: This runs in internal gstreamer thread
  void HandleDeepNotify(GstObject* gstobject, GstObject* prop_object,
                        GParamSpec* prop) {
    // Code from implementation of gst_object_default_deep_notify
    // This is what happens in gst-launch with -v option
    if (! (prop->flags & G_PARAM_READABLE)) {
      // Unreadable parameter -- ignore
      return;
    }
    if (strcmp(prop->name, "caps") == 0 ||
        strcmp(prop->name, "device") == 0 ||
        strcmp(prop->name, "num-buffers") == 0) {
      // caps are just too verbose...
      return;
    }

    // only select device-fd field.
    if (strcmp(prop->name, "device-fd") == 0 &&
        prop->value_type == G_TYPE_INT) {
      gint dev_fd = -1;
      g_object_get(prop_object, "device-fd", &dev_fd, NULL);
      log_.debug("uvch264src camera has fd %d", dev_fd);
      return;
    }

    if (bus_log_.isDebugEnabled()) {
      GValue value = { 0, };
      g_value_init(&value, prop->value_type);
      g_object_get_property(G_OBJECT(prop_object), prop->name, &value);
      // Can also do g_value_dup_string(&value) when
      // G_VALUE_HOLDS_STRING(&value)
      gchar* str = gst_value_serialize (&value);
      char* obj_name = gst_object_get_path_string(prop_object);
      bus_log_.debug("camera-recv deep notify %s: %s = %s",
                     obj_name, prop->name, str);
      g_free (obj_name);
      g_free (str);
      g_value_unset (&value);
    }
  }

  // WARNING: This runs in internal gstreamer thread
  void HandleRawSinkNewSample(GstSample* sample) {
    //GstCaps* caps = gst_sample_get_caps(sample);
    //GstBuffer* buf = gst_sample_get_buffer(sample);

    for (std::weak_ptr<CameraFrameConsumer>& c_weak: consumers_) {
      std::shared_ptr<CameraFrameConsumer> c = c_weak.lock();
      if (!c) {
        log_.debug("frame consumer gone -- not delivering raw sample");
        continue;
      }
      c->ConsumeRawSample(sample);
    }

    int count;
    {
      std::lock_guard<std::mutex> guard(stats_mutex_);
      count = stats_->raw_frames++;
    }

    const int kInterval = 20;
    if (video_test_src_ &&
        (count % kInterval) == 0) {
      // in test mode, periodically change background
      guint background =
          ((count / kInterval) % 2) ? 0 : 0x800080;
      g_object_set(video_test_src_,
                   "background-color", background,
                   NULL);
    }
  }

  // WARNING: This runs in internal gstreamer thread
  void HandleH264SinkNewSample(GstSample* sample) {
    //GstCaps* caps = gst_sample_get_caps(sample);
    GstBuffer* buf = gst_sample_get_buffer(sample);
    int len = gst_buffer_get_size(buf);
    bool key_frame = !GST_BUFFER_FLAG_IS_SET(buf, GST_BUFFER_FLAG_DELTA_UNIT);

    // no need to unref buffer -- it is held by sample

    for (std::weak_ptr<CameraFrameConsumer>& c_weak: consumers_) {
      std::shared_ptr<CameraFrameConsumer> c = c_weak.lock();
      if (!c) {
        log_.debug("frame consumer gone -- not delivering h264 sample");
        continue;
      }
      c->ConsumeH264Sample(sample);
    }

    GstCaps* caps = gst_sample_get_caps(sample);
    assert(GST_CAPS_IS_SIMPLE(caps));
    if (!(last_h264_caps_ && gst_caps_is_equal(caps, last_h264_caps_))) {
      char* caps_str = gst_caps_to_string(caps);
      log_.debug("H264 caps changed (frame %d): %s",
                 total_h264_frames_, caps_str);
      g_free(caps_str);
      gst_caps_ref(caps);
      last_h264_caps_ = caps;
    }

    total_h264_frames_++;

    int flags = GST_BUFFER_FLAGS(buf);
    int unusual_flags = flags &~ GST_BUFFER_FLAG_DELTA_UNIT;
    if ((total_h264_frames_ <= 1) || !dumb_camera_) {
      // somehow, our camera alwasy reports DISCONT flag
      unusual_flags &=~ GST_BUFFER_FLAG_DISCONT;
    }
    if (unusual_flags) {
      log_.notice("unusual buffer flags 0x%X (total 0x%X) in frame %d",
                  unusual_flags, flags, total_h264_frames_);
    }

    double interval = 0;
    const auto now = base::Now(parent_service_);
    if (last_h264_time_) {
      interval = base::ConvertDurationToSeconds(now - *last_h264_time_);
    }
    last_h264_time_ = now;

    {
      std::lock_guard<std::mutex> guard(stats_mutex_);
      stats_->h264_frames++;
      stats_->h264_bytes += len;
      stats_->h264_max_bytes = std::max(
          stats_->h264_max_bytes, len);
      stats_->h264_max_interval_s = std::max(
          stats_->h264_max_interval_s, interval);
      if (key_frame) { stats_->h264_key_frames++; };
    }
  }

  void HandleStatsTimeout() {
    BOOST_ASSERT(std::this_thread::get_id() == gst_loop_->thread_id());

    std::shared_ptr<CameraStats> other(new CameraStats());
    {
      std::lock_guard<std::mutex> guard(stats_mutex_);
      std::swap(stats_, other);
    }

    other->timestamp = base::Now(parent_service_);

    for (std::weak_ptr<CameraFrameConsumer>& c_weak: consumers_) {
      std::shared_ptr<CameraFrameConsumer> c = c_weak.lock();
      if (!c) {
        log_.warn("frame consumer gone -- not asking for stats");
        continue;
      }
      c->PreEmitStats(other.get());
    }

    parent_service_.post(std::bind(&CameraDriver::Impl::HandleStatsMainThread,
                                   this, other));
  }

  void HandleStatsMainThread(std::shared_ptr<CameraStats> stats) {
    BOOST_ASSERT(std::this_thread::get_id() == parent_id_);
    if (stats_log_.isDebugEnabled()) {
      stats_log_.debug(gst::FormatStatsForLogging(stats.get()));
    }
    parent_->camera_stats_signal_(stats.get());
  }

  // From both, changed only on startup
  CameraDriver* const parent_;
  boost::asio::io_service& parent_service_;

  Parameters parameters_;
  std::vector<std::weak_ptr<CameraFrameConsumer> > consumers_;
  bool started_ = false;
  base::LogRef log_ = base::GetLogInstance("camera_driver");
  base::LogRef stats_log_ = base::GetLogInstance("camera_driver.stats");
  base::LogRef bus_log_ = base::GetLogInstance("camera_driver.bus");

  const std::thread::id parent_id_;
  GstMainLoopRef gst_loop_;
  bool dumb_camera_ = false;
  std::unique_ptr<gst::PipelineWrapper> pipeline_;
  GstElement* video_test_src_ = nullptr;

  // From both, protected by mutex.
  std::mutex stats_mutex_;
  std::shared_ptr<CameraStats> stats_;

  // From child (and maybe gst threads) only.
  int total_h264_frames_ = 0;
  boost::optional<boost::posix_time::ptime> last_h264_time_;
  GstCaps* last_h264_caps_ = nullptr;
};


CameraDriver::CameraDriver(base::Context& context)
    : impl_(new Impl(this, context.service)) {
  context.telemetry_registry->Register(
      "camera_stats", &camera_stats_signal_);
}

CameraDriver::~CameraDriver() {}

void CameraDriver::AddFrameConsumer(std::weak_ptr<CameraFrameConsumer> c) {
  impl_->AddFrameConsumer(c);
}

void CameraDriver::AsyncStart(base::ErrorHandler handler) {
  impl_->AsyncStart(handler);
}

void CameraDriver::HandleGstReady(GstMainLoopRef& loop_ref) {
  impl_->HandleGstReady(loop_ref);
}

}
}

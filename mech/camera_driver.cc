// Copyright 2014-2015 Mikhail Afanasyev.  All rights reserved.
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

#include "gst_helpers.h"

namespace mjmech {
namespace mech {

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

    if (is_test) {
      out << "videotestsrc is-live=1 pattern=ball ";
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
      if (parameters_.bitrate_Bps > 0) {
        int bps = (parameters_.bitrate_Bps * 8);
        out << " average-bitrate=" << bps
            << " initial-bitrate=" << bps
            << " peak-bitrate=" << (bps * 1.5);
      }
      out << " " << parameters_.extra_uvch264
          << " src.vfsrc ";
    }

    // Make decoded image endpoint
    double decoded_fps =
        (parameters_.decoded_framerate > 0)
        ? parameters_.decoded_framerate
        : parameters_.framerate;
    out << "! video/x-raw,format=YUY2,framerate="
        << gst::FormatFraction(decoded_fps) << ","
        << parameters_.decoded_caps << " ";
    if (is_dumb) {
      out << "! tee name=dec-tee ";
    };
    out << "! queue ! appsink name=raw-sink ";

    // Make H264 endpoint
    if (is_dumb) {
      out << "dec-tee. ! videoconvert ! queue "
          << " ! x264enc tune=zerolatency byte-stream=true ";
      if (parameters_.iframe_interval_s) {
        int key_int = 0.5 + decoded_fps * parameters_.iframe_interval_s;
        out << "key-int-max=" << std::max(1, key_int) << " ";
      }
      if (parameters_.bitrate_Bps > 0) {
        out << "bitrate=" << (parameters_.bitrate_Bps * 8 / 1024) << " ";
      }
      out << " ! video/x-h264,framerate="
          << gst::FormatFraction(parameters_.framerate) << " ";
    } else {
      out << "src.vidsrc ! video/x-h264,framerate="
          << gst::FormatFraction(parameters_.framerate) << ","
          << parameters_.h264_caps << " ! h264parse ";
    };

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
      out << "! queue ! " << muxer
          << " ! filesink name=h264writer location="
          << gst::PipelineEscape(parameters_.write_video)
          << " h264-tee. ";
    }
    // Maybe serve it to custom consumer
    if (parameters_.custom_h264_consumer != "") {
      out << "! queue ! " << parameters_.custom_h264_consumer
          << " h264-tee. ";
    }
    // And pass it to our app
    out << "! queue ! appsink name=h264-sink";
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

    pipeline_->Start();

    // Set up the timer for stats
    if (parameters_.stats_interval_s > 0) {
      gst_loop_->AddPeriodicTimer(
           parameters_.stats_interval_s,
           std::bind(&Impl::HandleStatsTimeout, this));
    }
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

    {
      std::lock_guard<std::mutex> guard(stats_mutex_);
      stats_->raw_frames++;
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
    const auto now = boost::posix_time::microsec_clock::universal_time();
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

    other->timestamp = boost::posix_time::microsec_clock::universal_time();

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

  // From both.
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

  // From child (and maybe gst threads) only.
  std::unique_ptr<gst::PipelineWrapper> pipeline_;

  std::mutex stats_mutex_;
  std::shared_ptr<CameraStats> stats_;
  int total_h264_frames_ =0;
  bool dumb_camera_ = false;
  boost::optional<boost::posix_time::ptime> last_h264_time_;
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

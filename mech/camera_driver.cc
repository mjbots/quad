// Copyright 2014-2016 Mikhail Afanasyev.  All rights reserved.
// Copyright 2019 Josh Pieper, jjp@pobox.com.
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
#include <optional>
#include <thread>

#include <gst/gst.h>

#include <boost/algorithm/string/predicate.hpp>

#include <fmt/format.h>

#include "mjlib/base/fail.h"

#include "base/common.h"
#include "base/context_full.h"
#include "base/logging.h"
#include "base/now.h"

#include "gst_helpers.h"

namespace mjmech {
namespace mech {

namespace {
const double kFrameRate = 30.0;

struct Size {
  int width = 0;
  int height = 0;

  Size() {}
  Size(int width_in, int height_in) : width(width_in), height(height_in) {}
};

struct Bitrate {
  int bitrate_kbps = -1;
  double peak_scale = 1.5;
  double iframe_interval_s = 1.0;

  Bitrate() {}
  Bitrate(int bitrate_kbps_in, double peak_scale_in,
          double iframe_interval_s_in)
      : bitrate_kbps(bitrate_kbps_in),
        peak_scale(peak_scale_in),
        iframe_interval_s(iframe_interval_s_in) {}
};

struct Preset {
  int bitrate_kbps;
  Size h264_size;
  const char* extra_uvch264;
};

// To get list of supported resolutions, use: v4l2-ctl --list-formats-ext
const Preset kPresets[] = {{
 bitrate_kbps      : 3000,
 h264_size         : { 1920, 1080 },
 extra_uvch264     : "",
  }, {
 bitrate_kbps      : 700,
 h264_size         : { 1280, 720 },
 extra_uvch264     : "",
  }, {
 bitrate_kbps      : 750,
 h264_size         : { 640, 480 },
 extra_uvch264     : "",
  }, {
 bitrate_kbps      : 250,
 h264_size         : { 320, 240 },
 extra_uvch264     : "",
}};

}

class Device {
 public:
  enum class Type {
    kTest,
    kJoule,
    kC920,
    kGstreamer,
    kV4l2,
    kRaspberryPi,
  };

  Device(const std::string& name, const Size& size, const Bitrate& bitrate)
      : size_(size),
        bitrate_(bitrate) {
    if (name == "TEST") {
      type_ = Type::kTest;
    } else if (name == "JOULE") {
      type_ = Type::kJoule;
    } else if (boost::starts_with(name, "c920:")) {
      type_ = Type::kC920;
      data_ = boost::erase_first_copy(name, "c920:");
    } else if (boost::starts_with(name, "gstreamer:")) {
      type_ = Type::kGstreamer;
      data_ = boost::erase_first_copy(name, "gstreamer:");
    } else if (boost::starts_with(name, "v4l2:")) {
      type_ = Type::kV4l2;
      data_ = boost::erase_first_copy(name, "v4l2:");
    } else if (boost::starts_with(name, "rpi:")) {
      type_ = Type::kRaspberryPi;
      data_ = boost::erase_first_copy(name, "rpi:");
    } else {
      mjlib::base::Fail("Unknown device type: " + name);
    }
  }

  /// Return true if the primary stream is h264 encoded already.
  bool is_primary_h264() const {
    switch (type_) {
      case Type::kTest:
      case Type::kJoule:
      case Type::kGstreamer:
      case Type::kV4l2: {
        return false;
      }
      case Type::kC920:
      case Type::kRaspberryPi: {
        return true;
      }
    }
    mjlib::base::AssertNotReached();
  }

  /// This is required to return a gstreamer source which emits two
  /// separate streams, one on the un-nammed current stream which may
  /// or may not be h264 encoded already, and a secondary one on the
  /// decoded_pad(), which must be decoded.
  std::string gstreamer_source() const {
    const std::string final_tee = "! tee name=dec-tee ";
    switch (type_) {
      case Type::kTest: {
        return "videotestsrc name=testsrc is-live=1 pattern=ball " + final_tee;
      }
      case Type::kJoule: {
        return
            fmt::format(
                "icamerasrc device-name=0 io-mode=3 ! "
                "video/x-raw,format=NV12,width={},height={} "
                "! vaapipostproc dmabuf-alloc-tiled=true " + final_tee,
                size_.width, size_.height);
      }
      case Type::kC920: {
        std::ostringstream result;
        result <<
            fmt::format(
                "uvch264src device={} name=src auto-start=true "
                "message-forward=true ",
                gst::PipelineEscape(data_));
        // Desired I-frame interval for C920. 0 for camera-default. 1.0
        // seems to be the smallest possible -- any smaller numbers just
        // cause the setting to be ignored.
        if (bitrate_.iframe_interval_s > 0.0) {
          result <<
              fmt::format(
                  "iframe-period={} ",
                  static_cast<int>(bitrate_.iframe_interval_s * 1000.0));
        }
        if (bitrate_.bitrate_kbps > 0) {
          result <<
              fmt::format(
                  "average-bitrate={} initial-bitrate={} peak-bitrate={} ",
                  (bitrate_.bitrate_kbps * 1000),
                  (bitrate_.bitrate_kbps * 1000),
                  (bitrate_.bitrate_kbps * bitrate_.peak_scale * 1000));
        }
        result << "src.vidsrc";
        return result.str();
      }
      case Type::kGstreamer: {
        return data_;
      }
      case Type::kV4l2: {
        return
            fmt::format(
                "v4l2src name=src device={} ! videoconvert " + final_tee,
                gst::PipelineEscape(data_));
      }
      case Type::kRaspberryPi: {
        return
            fmt::format(
                "rpicamsrc bitrate={} "
                "keyframe-interval={} "
                "intra-refresh-type=cyclic "
                "inline-headers=true "
                " {} "
                " ! "
                "video/x-h264,width={},height={} ! "
                "tee name=pi264-tee "
                "pi264-tee. ! h264parse ! avdec_h264 ! videoconvert ! "
                "tee name=dec-tee pi264-tee. ",
                bitrate_.bitrate_kbps * 1000,
                static_cast<int>(kFrameRate * bitrate_.iframe_interval_s),
                data_,
                size_.width, size_.height);
      }
    }
    mjlib::base::AssertNotReached();
  }

  std::string decoded_pad() const {
    switch (type_) {
      case Type::kTest:
      case Type::kJoule:
      case Type::kGstreamer:
      case Type::kV4l2:
      case Type::kRaspberryPi:
        return "dec-tee";
      case Type::kC920:
        return "src.vfsrc";
    }
    mjlib::base::AssertNotReached();
  }

 private:
  Type type_;
  std::string data_;
  Size size_;
  Bitrate bitrate_;
};

class H264Encoder {
 public:
  enum class Type {
    kX264,
    kVaapi,
    kGstreamer,
  };

  H264Encoder(const std::string& name,
              const Size& size,
              const Bitrate& bitrate)
      : size_(size), bitrate_(bitrate) {
    if (name == "X264") {
      type_ = Type::kX264;
    } else if (name == "VAAPI") {
      type_ = Type::kVaapi;
    } else if (boost::starts_with(name, "gstreamer:")) {
      type_ = Type::kGstreamer;
      data_ = boost::erase_first_copy(name, "gstreamer:");
    } else {
      mjlib::base::Fail("Unknown encoder type: " + name);
    }
  }

  std::string gstreamer_element() const {
    const int key_int =
        std::max(1, static_cast<int>(
                     0.5 + kFrameRate * bitrate_.iframe_interval_s));

    switch (type_) {
      case Type::kX264: {
        std::ostringstream ostr;
        ostr << "! queue ! x264enc byte-stream=true speed-preset=ultrafast"
            " intra-refresh=true threads=1 sync-lookahead=0"
            " bframes=0 tune=zerolatency ";

        if (bitrate_.iframe_interval_s > 0.0) {
          ostr << fmt::format("key-int-max={} ", key_int);
        }

        return ostr.str();
      }
      case Type::kVaapi: {
        std::ostringstream ostr;
        ostr <<
            fmt::format(
                "! vaapih264enc bitrate={} tune=none rate-control=vbr ",
                bitrate_.bitrate_kbps);
        if (bitrate_.iframe_interval_s > 0.0) {
          ostr << fmt::format("keyframe-period={} ", key_int);
        }

        return ostr.str();
      }
      case Type::kGstreamer: {
        return data_;
      }
    }
    mjlib::base::AssertNotReached();
  }

 private:
  Type type_;
  std::string data_;
  Size size_;
  Bitrate bitrate_;
};

class CameraDriver::Impl : boost::noncopyable {
 public:
  Impl(CameraDriver* parent, boost::asio::io_context& service)
      : parent_(parent),
        parent_service_(service),
        parent_id_(std::this_thread::get_id()),
        stats_(new CameraStats()) {}

  ~Impl() {
    if (gst_loop_) {
      gst_loop_->WaitForQuit();
    }
  }

  void AsyncStart(mjlib::io::ErrorCallback handler) {
    BOOST_ASSERT(std::this_thread::get_id() == parent_id_);
    BOOST_ASSERT(!gst_loop_);

    started_ = true;
    parameters_ = parent_->parameters_;

    // Apply presets
    const int kMaxPresets = sizeof(kPresets) / sizeof(kPresets[0]) - 1;

    int i_preset = parameters_.preset;
    mjlib::base::error_code error;

    if (i_preset < 0 || i_preset > kMaxPresets) {
      std::string msg = fmt::format("Preset {} is not in the range 0..{}",
                                    i_preset, kMaxPresets);
      parent_service_.post(std::bind(handler, mjlib::base::error_code::einval(msg)));
      return;
    }
    const Preset& preset = kPresets[i_preset];

    if (parameters_.bitrate_kbps < 0) {
      parameters_.bitrate_kbps = preset.bitrate_kbps;
    }
    if (parameters_.h264_width < 0) {
      parameters_.h264_width = preset.h264_size.width;
    }
    if (parameters_.h264_height < 0) {
      parameters_.h264_height = preset.h264_size.height;
    }

    parent_service_.post(std::bind(handler, mjlib::base::error_code()));
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
    if (!parameters_.gstreamer_pipeline.empty()) {
      return parameters_.gstreamer_pipeline;
    }

    std::ostringstream out;

    const Size size(parameters_.h264_width, parameters_.h264_height);
    const Bitrate bitrate(parameters_.bitrate_kbps * parameters_.bitrate_scale,
                          parameters_.bitrate_peak_scale,
                          parameters_.iframe_interval_s);

    Device device(parameters_.device, size, bitrate);

    out << device.gstreamer_source();

    if (!device.is_primary_h264()) {
      // Do h264 encoding.
      H264Encoder encoder(parameters_.h264_encoder, size, bitrate);
      out << encoder.gstreamer_element();
    }


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

    out << "! h264parse ! appsink name=h264-sink ";
    out << "caps=video/x-h264,stream-format=byte-stream,alignment=au,";
    out << "profile=constrained-baseline ";

    // Now let's switch and get our decoded data to our app.

    out << (device.decoded_pad() + ". ");

    if (parameters_.analyze) {
      out << "! videoanalyse name=input-analyze ";
    }
    out << "! queue max-size-buffers=1 ! appsink name=raw-sink ";

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
  void HandleDeepNotify(GstObject*, GstObject* prop_object,
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
      GValue value = {};
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
    unusual_flags = unusual_flags &~ GST_BUFFER_FLAG_TAG_MEMORY;
    if ((total_h264_frames_ <= 1) || !dumb_camera_) {
      // somehow, our camera alwasy reports these flags
      unusual_flags &=~ GST_BUFFER_FLAG_DISCONT;
    }
    if (unusual_flags) {
      log_.notice("unusual buffer flags   0x%X (total   0x%X) in frame   %d",
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
  boost::asio::io_context& parent_service_;

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
  std::optional<boost::posix_time::ptime> last_h264_time_;
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

void CameraDriver::AsyncStart(mjlib::io::ErrorCallback handler) {
  impl_->AsyncStart(handler);
}

void CameraDriver::HandleGstReady(GstMainLoopRef& loop_ref) {
  impl_->HandleGstReady(loop_ref);
}

}
}

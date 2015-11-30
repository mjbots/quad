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
#include <gst/app/gstappsink.h>

#include <boost/format.hpp>

#include "base/common.h"
#include "base/fail.h"
#include "base/json_archive.h"
#include "base/logging.h"
#include "base/property_tree_archive.h"

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
    loop_ref->quit_request_signal()->connect(
        std::bind(&Impl::HandleShutdown, this, std::placeholders::_1));
    SetupPipeline();
  }

 private:
  // Escape string for gstreamer pipeline
  static std::string pp_escape(const std::string& s) {
    if (s.find(' ') == std::string::npos) {
      return s;
    }
    base::Fail("string escaping not implemented");
  }

  // Format floating point number as fraction (10/1)
  static std::string FormatFraction(double val) {
    BOOST_ASSERT(val > 0);
    gint n = 0, d = 0;
    gst_util_double_to_fraction(val, &n, &d);
    return (boost::format("%d/%d") % n % d).str();
  }

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
      out << "v4l2src name=src device=" << pp_escape(device)
          << " ! videoconvert ";
    } else {
      out << "uvch264src device=" << pp_escape(device)
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
        << FormatFraction(decoded_fps) << ","
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
          << FormatFraction(parameters_.framerate) << " ";
    } else {
      out << "src.vidsrc ! video/x-h264,framerate="
          << FormatFraction(parameters_.framerate) << ","
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
          << pp_escape(parameters_.write_video)
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

    // Create a gstreamer pipeline
    GError* error = NULL;
    pipeline_ = gst_parse_launch(launch_cmd.c_str(), &error);
    if (!pipeline_ || error) {
      base::Fail(
          boost::format("Failed to launch gstreamer pipeline: error %d: %s"
                        "\nPipeline command was: %s\n")
          % error->code % error->message % launch_cmd);
    }
    BOOST_ASSERT(error == NULL);

    // Hook the global bus
    GstBus* bus = gst_element_get_bus(pipeline_);
    gst_bus_add_watch(bus, handle_bus_message_wrapper, this);
    gst_object_unref(bus);

    // Hook the deep nitification
    g_signal_connect(pipeline_, "deep-notify",
                     G_CALLBACK(handle_deep_notify_wrapper), this);

    // Hook / setup raw data sink
    GstAppSink* raw_sink = GST_APP_SINK(
       gst_bin_get_by_name_recurse_up(GST_BIN(pipeline_), "raw-sink"));
    if (raw_sink == NULL) {
      base::Fail("could not find raw-sink element");
    }
    gst_app_sink_set_emit_signals(raw_sink, TRUE);
    gst_app_sink_set_max_buffers(raw_sink, 1);
    gst_app_sink_set_drop(raw_sink, TRUE);
    g_signal_connect(raw_sink, "new-sample",
                     G_CALLBACK(raw_sink_new_sample_wrapper), this);
    gst_object_unref(raw_sink);

    // Hook / setup h264 data sink
    GstAppSink* h264_sink = GST_APP_SINK(
       gst_bin_get_by_name_recurse_up(GST_BIN(pipeline_), "h264-sink"));
    if (raw_sink == NULL) {
      base::Fail("could not find h264-sink element");
    }
    gst_app_sink_set_emit_signals(h264_sink, TRUE);
    gst_app_sink_set_max_buffers(h264_sink, 120);
    gst_app_sink_set_drop(h264_sink, TRUE);
    g_signal_connect(h264_sink, "new-sample",
                     G_CALLBACK(h264_sink_new_sample_wrapper), this);
    gst_object_unref(h264_sink);

    log_.debug("created pipeline with cmd: " + launch_cmd);

    // Start the pipeline
    // TODO theamk: only when autostart == true?
    GstStateChangeReturn rv =
      gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (rv == GST_STATE_CHANGE_FAILURE) {
      // no need to exit here -- we will get an error message on the bus with
      // more information.
      log_.error("Failed to start pipeline: %d", rv);
    }

    // Set up the timer for stats
    if (parameters_.stats_interval_s > 0) {
      g_timeout_add(parameters_.stats_interval_s * 1000.0,
                    stats_timeout_wrapper, this);
    }
  }


  static gboolean handle_bus_message_wrapper(
      GstBus *bus, GstMessage *message, gpointer user_data) {
    return static_cast<CameraDriver::Impl*>(user_data)
      ->HandleBusMessage(bus, message) ? TRUE : FALSE;
  }

  bool HandleBusMessage(GstBus *bus, GstMessage *message) {
    BOOST_ASSERT(std::this_thread::get_id() == gst_loop_->thread_id());

    bool print_message = true;
    switch (GST_MESSAGE_TYPE(message)) {
    case GST_MESSAGE_EOS: {
      if (!quit_request_) {
        base::Fail("Unexpected EOS on pipeline");
      };
      log_.debug("got EOS -- stopping pipeline");
      GstStateChangeReturn rv =
          gst_element_set_state(pipeline_, GST_STATE_NULL);
      // note: 0 is failure, 1 is success, 2 is async (=fill do later)
      // note: when going to STATE_NULL, we will never get _ASYNC
      BOOST_ASSERT(rv == GST_STATE_CHANGE_SUCCESS);
      quit_request_.reset();
      print_message = false;
      break;
    }
    case GST_MESSAGE_ERROR: {
      std::string error_msg = "(no error)";
      GError *err = NULL;
      gchar *dbg = NULL;
      gst_message_parse_error(message, &err, &dbg);
      if (err) {
        error_msg = (boost::format("%s\nDebug details: %s")
                     % err->message % (dbg ? dbg : "(NONE)")).str();
        g_error_free(err);
      }
      if (dbg) { g_free(dbg); }

      base::Fail("gstreamer pipeline error: " + error_msg);
      break;
    }
    case GST_MESSAGE_STATE_CHANGED:
    case GST_MESSAGE_STREAM_STATUS:
    case GST_MESSAGE_TAG:
    case GST_MESSAGE_NEW_CLOCK:
    case GST_MESSAGE_ASYNC_DONE: {
      // Ignore
      print_message = false;
      break;
    }
    default: { break; };
    }
    if (print_message && bus_log_.isDebugEnabled()) {
      const GstStructure* mstruct = gst_message_get_structure(message);
      char* struct_info =
        mstruct ? gst_structure_to_string(mstruct) : g_strdup("no-struct");
      bus_log_.debugStream() <<
          boost::format("message '%s' from '%s': %s") %
          GST_MESSAGE_TYPE_NAME(message) % GST_MESSAGE_SRC_NAME(message)
          % struct_info;
      g_free(struct_info);
    }
    return true;
  }

  static gboolean handle_deep_notify_wrapper(
      GstObject* gstobject, GstObject* prop_object,
      GParamSpec* prop, gpointer user_data) {
    static_cast<CameraDriver::Impl*>(user_data)
        ->HandleDeepNotify(gstobject, prop_object, prop);
    return TRUE;
  }

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

  static GstFlowReturn raw_sink_new_sample_wrapper(GstElement* sink,
                                           gpointer user_data) {
    return static_cast<CameraDriver::Impl*>(user_data)
      ->HandleRawSinkNewSample(sink);
  }

  // WARNING: This runs in internal gstreamer thread
  GstFlowReturn HandleRawSinkNewSample(GstElement* sink) {
    GstSample* sample = NULL;
    g_signal_emit_by_name(sink, "pull-sample", &sample);
    if (!sample) {
      log_.error("raw sink is out of samples");
      return GST_FLOW_OK;
    }

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
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }

  static GstFlowReturn h264_sink_new_sample_wrapper(GstElement* sink,
                                           gpointer user_data) {
    return static_cast<CameraDriver::Impl*>(user_data)
      ->HandleH264SinkNewSample(sink);
  }

  // WARNING: This runs in internal gstreamer thread
  GstFlowReturn HandleH264SinkNewSample(GstElement* sink) {
    GstSample* sample = NULL;
    g_signal_emit_by_name(sink, "pull-sample", &sample);
    if (!sample) {
      base::Fail("warning: h264 sink is out of samples");
      return GST_FLOW_OK;
    }

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
      stats_->h264_max_interval_s = std::max(
          stats_->h264_max_interval_s, interval);
      if (key_frame) { stats_->h264_key_frames++; };
    }

    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }

  static gboolean stats_timeout_wrapper(gpointer user_data) {
    static_cast<CameraDriver::Impl*>(user_data)->HandleStatsTimeout();
    return TRUE;
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
      std::ostringstream out;
      //base::JsonWriteArchive(out).Accept(other.get());
      base::PropertyTreeWriteArchive arch;
      arch.Accept(stats.get());
      for (const auto& elem: arch.tree()) {
        if (elem.first == "timestamp") { continue; }
        if (elem.second.data() == "0") { continue; }
        if (out.tellp()) { out << " "; }
        out << elem.first << "=" << elem.second.data();
      }
      if (out.tellp() == 0) {
        out << "(no data)";
      }
      stats_log_.debug(out.str());
    }

    parent_->camera_stats_signal_(stats.get());
  }

  void HandleShutdown(GstMainLoopRefObj::QuitPostponerPtr& ptr) {
    BOOST_ASSERT(std::this_thread::get_id() == gst_loop_->thread_id());
    BOOST_ASSERT(!quit_request_);
    quit_request_ = ptr;
    // send 'end of stream' event
    gst_element_send_event(pipeline_, gst_event_new_eos());
    log_.debug("shutdown requested -- sending end-of-stream");
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
  GstMainLoopRefObj::QuitPostponerPtr quit_request_;

  // From child (and maybe gst threads) only.
  GstElement* pipeline_ = NULL;
  std::mutex stats_mutex_;
  std::shared_ptr<CameraStats> stats_;
  int total_h264_frames_ =0;
  bool dumb_camera_ = false;
  boost::optional<boost::posix_time::ptime> last_h264_time_;
};


CameraDriver::CameraDriver(boost::asio::io_service& service)
  : impl_(new Impl(this, service)) {};

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

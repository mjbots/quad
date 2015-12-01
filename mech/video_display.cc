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

#include "video_display.h"

#include <mutex>
#include <thread>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/format.hpp>

#include "base/common.h"
#include "base/fail.h"
#include "base/json_archive.h"
#include "base/logging.h"
#include "base/property_tree_archive.h"

namespace mjmech {
namespace mech {

// TODO theamk: need to factor out common gst-pipeline support from
// both VideoDisplay and CameraDriver.


// Note: The 'identiy' elements are not required -- we could have
// achieved the same effect with pad probes. I just think that identity
// elements are slightly easier to understand.
class IdentityHandoffHandler : boost::noncopyable {
 public:
  typedef std::function<void(GstBuffer*)> Callback;

  IdentityHandoffHandler(GstElement* pipeline,
                         const char* element_name,
                         Callback cb)
      : callback_(cb) {
    GstElement* target =
        gst_bin_get_by_name_recurse_up(GST_BIN(pipeline), element_name);
    if (!target) {
      base::Fail(boost::format("Cannot find identity element %s in a pipeline")
                 % element_name);
    }
    int id = g_signal_connect(target, "handoff",
                              G_CALLBACK(handoff_wrapper), this);
    BOOST_ASSERT(id > 0);
    gst_object_unref(target);
  }

 private:
  Callback callback_;
  static void handoff_wrapper(GstElement*, GstBuffer* buffer,
                              gpointer user_data) {
    static_cast<IdentityHandoffHandler*>(user_data)
        ->callback_(buffer);
  }
};

class VideoDisplay::Impl : boost::noncopyable {
 public:
  Impl(VideoDisplay* parent, boost::asio::io_service& service)
      : parent_(parent),
        parent_service_(service),
        parent_id_(std::this_thread::get_id()),
        stats_(new Stats()) {}

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

    // Get raw packets
    std::string source = parameters_.source;
    bool is_test = (source == "TEST");
    if (is_test) {
      out << "videotestsrc is-live=1 pattern=ball ! ";
      //"video/x-raw,format=YUY2,width=1920,height=1080,framerate=15/1"
      out << "video/x-raw,width=640,height=480,framerate=15/1"
          << " ! queue ! x264enc tune=zerolatency byte-stream=true ";
    } else if (boost::starts_with(source, "rtsp://")) {
      out << "rtspsrc location=" << pp_escape(source)
          <<" latency=200 ";
    } else if (source != "") {
      out << source << " ";
    } else {
      out << "appsrc name=raw-src ";
    }

    // Make them into h264 frames
    out << "! queue ! identity name=raw-detector silent=false "
        << "! h264parse ! identity name=h264-detector silent=false ";

    // Maybe save them
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
      out << "! tee name=h264-tee ! queue ! " << muxer
          << " ! filesink name=h264writer location="
          << pp_escape(parameters_.write_video)
          << " h264-tee. ";
    }

    // Decode and do some basic overlays.
    out << "! avdec_h264 ! videoconvert "
        << "! timeoverlay shaded_background=1 font_desc=8 valignment=bottom "
        << "   halignment=right "
        << "! identity name=decoded-detector silent=false ";

    // Maybe pass it to our app for OSD.
    if (parameters_.process_frames) {
      out << "! queue ! appsink name=decoded-sink "
          << " appsrc name=decoded-src ";
    }

    if (parameters_.hide_video) {
      out << "! fakesink sync=false";
    } else {
      out << "! xvimagesink sync=false";
    }

    return out.str();
  }

  void SetupPipeline() {
    std::string launch_cmd = MakeLaunchCmd();

    log_.debugStream() << "creating pipeline " << launch_cmd;

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

    // Hook the counters. Note: we should have stored them somewhere
    // if we wanted to release the memory properly.
    new IdentityHandoffHandler(
        pipeline_, "raw-detector", [this](GstBuffer* buf) {
          std::lock_guard<std::mutex> guard(stats_mutex_);
          stats_->raw_frames++;
          stats_->raw_bytes += gst_buffer_get_size(buf);
        });
    new IdentityHandoffHandler(
        pipeline_, "h264-detector", [this](GstBuffer* buf) {
          std::lock_guard<std::mutex> guard(stats_mutex_);
          stats_->h264_frames++;
          if (!GST_BUFFER_FLAG_IS_SET(buf, GST_BUFFER_FLAG_DELTA_UNIT)) {
            stats_->h264_key_frames++;
          }
        });
    new IdentityHandoffHandler(
        pipeline_, "decoded-detector", [this](GstBuffer* buf) {
          std::lock_guard<std::mutex> guard(stats_mutex_);
          stats_->decoded_frames++;

          const auto now = boost::posix_time::microsec_clock::universal_time();
          if (last_decoded_time_) {
            stats_->decoded_max_interval_s = std::max(
                stats_->decoded_max_interval_s,
                base::ConvertDurationToSeconds(now - *last_decoded_time_));
          }
          last_decoded_time_ = now;
        });

    // Start the pipeline
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
    return static_cast<VideoDisplay::Impl*>(user_data)
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

  static gboolean stats_timeout_wrapper(gpointer user_data) {
    static_cast<VideoDisplay::Impl*>(user_data)->HandleStatsTimeout();
    return TRUE;
  }

  void HandleStatsTimeout() {
    BOOST_ASSERT(std::this_thread::get_id() == gst_loop_->thread_id());

    std::shared_ptr<Stats> other(new Stats());
    {
      std::lock_guard<std::mutex> guard(stats_mutex_);
      std::swap(stats_, other);
    }

    other->timestamp = boost::posix_time::microsec_clock::universal_time();

    parent_service_.post(std::bind(&VideoDisplay::Impl::HandleStatsMainThread,
                                   this, other));
  }

  void HandleStatsMainThread(std::shared_ptr<Stats> stats) {
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

    parent_->stats_signal_(stats.get());
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
  VideoDisplay* const parent_;
  boost::asio::io_service& parent_service_;

  Parameters parameters_;
  bool started_ = false;
  base::LogRef log_ = base::GetLogInstance("video_display");
  base::LogRef stats_log_ = base::GetLogInstance("video_display.stats");
  base::LogRef bus_log_ = base::GetLogInstance("video_display.bus");

  const std::thread::id parent_id_;
  GstMainLoopRef gst_loop_;
  GstMainLoopRefObj::QuitPostponerPtr quit_request_;

  // From child (and maybe gst threads) only.
  GstElement* pipeline_ = NULL;

  std::mutex stats_mutex_;
  std::shared_ptr<Stats> stats_;
  boost::optional<boost::posix_time::ptime> last_decoded_time_;

};


VideoDisplay::VideoDisplay(boost::asio::io_service& service)
  : impl_(new Impl(this, service)) {};

VideoDisplay::~VideoDisplay() {}

void VideoDisplay::AsyncStart(base::ErrorHandler handler) {
  impl_->AsyncStart(handler);
}

void VideoDisplay::HandleGstReady(GstMainLoopRef& loop_ref) {
  impl_->HandleGstReady(loop_ref);
}

}
}

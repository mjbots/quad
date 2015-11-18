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

#include "base/common.h"
#include "base/fail.h"
#include "base/json_archive.h"
#include "base/property_tree_archive.h"

namespace mjmech {
namespace mech {

namespace {

void gstreamer_global_init(const std::string& gst_options) {
  // prepare argc/argv for getstreamer
  // (the path in argv[0] may matter. so we set it to
  // bogus, but recognizeable value)
  std::string cmdline = "/mjmech-workdir/camera_driver";
  if (gst_options != "") {
    cmdline += " " + gst_options;
  }
  char ** gs_argv = g_strsplit(cmdline.c_str(), " ", 0);
  int gs_argc = g_strv_length(gs_argv);
  gst_init(&gs_argc, reinterpret_cast<char***>(&gs_argv));
  if (gs_argc != 1) {
    base::Fail(std::string("Unhandled gst option: ") +
               std::string(gs_argv[1]));
  }
  g_strfreev(gs_argv);

  char* version = gst_version_string();
  std::cerr << "gstreamer ready: " << version << "\n";
  g_free(version);

  // Exit on critical messages from our app
  g_log_set_fatal_mask(
       NULL,
       static_cast<GLogLevelFlags>(G_LOG_LEVEL_ERROR | G_LOG_LEVEL_CRITICAL));
  // Exit on critical messages from GStreamer
  g_log_set_fatal_mask(
       "GStreamer",
       static_cast<GLogLevelFlags>(G_LOG_LEVEL_ERROR | G_LOG_LEVEL_CRITICAL));
};

}


class CameraDriver::Impl : boost::noncopyable {
 public:
  Impl(CameraDriver* parent, boost::asio::io_service& service)
      : parent_(parent),
        service_(service),
        work_(service_),
        parent_id_(std::this_thread::get_id()),
        stats_(new CameraStats()) {}

  ~Impl() {
    done_ = true;
    if (loop_) {
      g_main_loop_quit(loop_);
    }
    if (child_.joinable()) { child_.join(); }
  }

  void AsyncStart(base::ErrorHandler handler) {
    BOOST_ASSERT(std::this_thread::get_id() == parent_id_);

    // Capture our parent's parameters before starting our thread.
    parameters_ = parent_->parameters_;

    child_ = std::thread(std::bind(&Impl::Run, this, handler));
  }

 private:

  std::string MakeLaunchCmd() {
    std::ostringstream out;
    // TODO: port from video-ui/vserver/video_tools/camera-recv.c
    out << "videotestsrc is-live=1 pattern=ball "
        << "! queue ! appsink name=raw-sink";
    return out.str();
  }

  void Run(base::ErrorHandler handler) {
    BOOST_ASSERT(std::this_thread::get_id() == child_.get_id());

    // Init global state
    gstreamer_global_init(parameters_.gst_options);
    loop_ = g_main_loop_new(NULL, FALSE);

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

    std::cerr << "created pipeline with cmd: " << launch_cmd << "\n";

    // Start the pipeline
    // TODO theamk: only when autostart == true?
    GstStateChangeReturn rv =
      gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (rv == GST_STATE_CHANGE_FAILURE) {
      base::Fail(boost::format("Failed to start pipeline: %d") % rv);
    }

    // Set up the timer for stats
    if (parameters_.stats_interval_ms > 0) {
      g_timeout_add(parameters_.stats_interval_ms,
                    stats_timeout_wrapper, this);
    }


    g_main_loop_run(loop_);
    if (done_) {
      std::cerr << "camera driver main loop quitting\n";
    } else {
      base::Fail("camera driver loop exited unexpectedly");
    }
  }


  static gboolean handle_bus_message_wrapper(
      GstBus *bus, GstMessage *message, gpointer user_data) {
    return static_cast<CameraDriver::Impl*>(user_data)
      ->HandleBusMessage(bus, message) ? TRUE : FALSE;
  }

  bool HandleBusMessage(GstBus *bus, GstMessage *message) {
    BOOST_ASSERT(std::this_thread::get_id() == child_.get_id());

    bool print_message = false;
    switch (GST_MESSAGE_TYPE(message)) {
    case GST_MESSAGE_EOS: {
      base::Fail("EOS on pipeline");
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
    case GST_MESSAGE_NEW_CLOCK: {
      // Ignore
      print_message = false;
      break;
    }
    default: { break; };
    }
    if (print_message) {
      const GstStructure* mstruct = gst_message_get_structure(message);
      char* struct_info =
        mstruct ? gst_structure_to_string(mstruct) : g_strdup("no-struct");
      std::cerr << boost::format("camera_driver message '%s' from '%s': %s") %
        GST_MESSAGE_TYPE_NAME(message) % GST_MESSAGE_SRC_NAME(message)
        % struct_info;
      g_free(struct_info);
    }
    return true;
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
      std::cerr << "warnign: raw sink is out of samples\n";
      return GST_FLOW_OK;
    }

    //GstCaps* caps = gst_sample_get_caps(sample);
    //GstBuffer* buf = gst_sample_get_buffer(sample);
    //std::cerr << "got sample\n";

    {
      std::lock_guard<std::mutex> guard(stats_mutex_);
      stats_->raw_frames++;
    }
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }

  static gboolean stats_timeout_wrapper(gpointer user_data) {
    static_cast<CameraDriver::Impl*>(user_data)->HandleStatsTimeout();
    return TRUE;
  }

  void HandleStatsTimeout() {
    BOOST_ASSERT(std::this_thread::get_id() == child_.get_id());

    std::shared_ptr<CameraStats> other(new CameraStats());
    {
      std::lock_guard<std::mutex> guard(stats_mutex_);
      std::swap(stats_, other);
    }

    other->timestamp = boost::posix_time::microsec_clock::universal_time();
    service_.post(std::bind(&CameraDriver::Impl::HandleStatsMainThread,
                            this, other));
  }

  void HandleStatsMainThread(std::shared_ptr<CameraStats> stats) {
    BOOST_ASSERT(std::this_thread::get_id() == parent_id_);

    if (parameters_.print_stats) {
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
      std::cout << "camera stats: " << out.str() << "\n";
    }

    parent_->camera_stats_signal_(stats.get());
  }

  // From both.
  CameraDriver* const parent_;
  boost::asio::io_service& service_;
  boost::asio::io_service::work work_;
  Parameters parameters_;

  const std::thread::id parent_id_;
  std::thread child_;
  bool done_ = false;
  GMainLoop* loop_ = NULL;

  // From child (and maybe gst threads) only.
  GstElement* pipeline_ = NULL;
  std::mutex stats_mutex_;
  std::shared_ptr<CameraStats> stats_;

};


CameraDriver::CameraDriver(boost::asio::io_service& service)
  : impl_(new Impl(this, service)) {};

CameraDriver::~CameraDriver() {}

void CameraDriver::AsyncStart(base::ErrorHandler handler) {
  impl_->AsyncStart(handler);
}

}
}

// Copyright 2015-2016 Mikhail Afanasyev.  All rights reserved.
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
#include "base/logging.h"
#include "base/now.h"

#include "gst_helpers.h"

namespace mjmech {
namespace mech {

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
    BOOST_ASSERT(!gst_loop_);

    started_ = true;
    parameters_ = parent_->parameters_;
  }

  void HandleGstReady(GstMainLoopRef& loop_ref) {
    gst_loop_ = loop_ref;
    if (!started_) {
      log_.debug("component disabled");
      return;
    }
    SetupPipeline();
  }

  void HandleIncomingFrame(std::shared_ptr<std::string>& frame) {
    if (!pipeline_ready_) {
      log_.debug("discarding frame -- not started yet");
      return;
    }
    if (!h264_src_) {
      base::Fail("got raw frame from video link, but other input is selected");
    }
    h264_src_(&frame->front(), frame->size());
  }

 private:
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
      out << "rtspsrc location=" << gst::PipelineEscape(source)
          <<" latency=200 ";
    } else if (source != "") {
      out << source << " ";
    } else {
      out << "appsrc name=raw-src ";
    }

    if (source != "") {
      // Make them into h264 frames
      out << "! queue ! identity name=raw-detector silent=false "
          << "! h264parse ";
    }
    out << "! identity name=h264-detector silent=false ";

    // Maybe save them
    if (parameters_.write_video != "") {
      out << "! tee name=h264-tee ! queue ! "
          << gst::MuxerForVideoName(parameters_.write_video)
          << " ! filesink name=h264writer location="
          << gst::PipelineEscape(parameters_.write_video)
          << " h264-tee. ";
    }

    // Decode and do some basic overlays.
    out << "! avdec_h264 ! videoconvert "
        << "! identity name=decoded-detector silent=false "
        << "! timeoverlay shaded_background=1 font_desc=8 valignment=bottom "
        << "   halignment=right ";

    // Maybe pass it to our app for OSD.
    if (parameters_.process_frames) {
      out << "! queue ! appsink name=decoded-sink "
          << " appsrc name=decoded-src ";
    }

    if (parameters_.analyze) {
      out << " ! videoanalyse name=final-analyze ";
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

    pipeline_.reset(
        new gst::PipelineWrapper(
            gst_loop_, "video_display", launch_cmd));

    // Hook the counters. Note: we should have stored them somewhere
    // if we wanted to release the memory properly.

    if (parameters_.source != "") {
      // If source is en empty string, we are getting data from
      // mcast_video_link, and it will handle our raw counter
      // for us.
      pipeline_->ConnectIdentityHandoff(
          "raw-detector", [this](GstBuffer* buf) {
            std::lock_guard<std::mutex> guard(stats_mutex_);
            if (parameters_.analyze) {
              log_.debug("detected raw frame %d", stats_->raw_frames);
            }
            stats_->raw_frames++;
            stats_->raw_bytes += gst_buffer_get_size(buf);
        });
    }

    pipeline_->ConnectIdentityHandoff(
        "h264-detector", [this](GstBuffer* buf) {
          std::lock_guard<std::mutex> guard(stats_mutex_);
          if (parameters_.analyze) {
            log_.debug("detected h264 frame %d", stats_->h264_frames);
          }
          stats_->h264_frames++;
          if (!GST_BUFFER_FLAG_IS_SET(buf, GST_BUFFER_FLAG_DELTA_UNIT)) {
            stats_->h264_key_frames++;
          }
        });

    pipeline_->ConnectIdentityHandoff(
        "decoded-detector", [this](GstBuffer* buf) {
          std::lock_guard<std::mutex> guard(stats_mutex_);
          if (parameters_.analyze) {
            log_.debug("detected decoded frame %d",
                       stats_->decoded_frames);
          }
          stats_->decoded_frames++;

          const auto now = base::Now(parent_service_);
          if (last_decoded_time_) {
            stats_->decoded_max_interval_s = std::max(
                stats_->decoded_max_interval_s,
                base::ConvertDurationToSeconds(now - *last_decoded_time_));
          }
          last_decoded_time_ = now;
        });

    if (parameters_.source == "") {
      h264_src_ = pipeline_->SetupAppsrc(
          "raw-src",
          "video/x-h264,stream-format=byte-stream,alignment=au,parsed=true");
    }

    pipeline_->RegisterVideoAnalyzeMessageHandler(
        "final-analyze",
        std::bind(&Impl::HandleVideoAnalyzeMessage,
                  this, std::placeholders::_1));

    pipeline_->Start();

    // Set up the timer for stats
    if (parameters_.stats_interval_s > 0) {
      gst_loop_->AddPeriodicTimer(
           parameters_.stats_interval_s,
           std::bind(&Impl::HandleStatsTimeout, this));
    }
    parent_service_.post([=]() { pipeline_ready_ = true; });
  }

  void HandleVideoAnalyzeMessage(const gst::VideoAnalyzeMessage& msg) {
    log_.infoStream() << "final frame info: "
                      << msg.toString();
  }

  void HandleStatsTimeout() {
    BOOST_ASSERT(std::this_thread::get_id() == gst_loop_->thread_id());

    std::shared_ptr<Stats> other(new Stats());
    {
      std::lock_guard<std::mutex> guard(stats_mutex_);
      std::swap(stats_, other);
    }

    other->timestamp = base::Now(parent_service_);

    parent_service_.post(std::bind(&VideoDisplay::Impl::HandleStatsMainThread,
                                   this, other));
  }

  void HandleStatsMainThread(std::shared_ptr<Stats> stats) {
    BOOST_ASSERT(std::this_thread::get_id() == parent_id_);

    parent_->pre_stats_signal_(stats.get());

    if (stats_log_.isDebugEnabled()) {
      stats_log_.debug(gst::FormatStatsForLogging(stats.get()));
    }

    parent_->stats_signal_(stats.get());
  }

  // From both.
  VideoDisplay* const parent_;
  boost::asio::io_service& parent_service_;

  Parameters parameters_;
  bool started_ = false;
  base::LogRef log_ = base::GetLogInstance("video_display");
  base::LogRef stats_log_ = base::GetLogInstance("video_display.stats");

  const std::thread::id parent_id_;
  GstMainLoopRef gst_loop_;

  bool pipeline_ready_ = false;
  gst::PipelineWrapper::AppsrcSampleCallback h264_src_ = nullptr;

  // From child (and maybe gst threads) only.
  std::unique_ptr<gst::PipelineWrapper> pipeline_;

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

void VideoDisplay::HandleIncomingFrame(std::shared_ptr<std::string>& data) {
  impl_->HandleIncomingFrame(data);
}

}
}

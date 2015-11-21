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

#include "rtsp_server.h"

#include <mutex>
#include <thread>
#include <atomic>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/rtsp-server/rtsp-server.h>

#include "base/common.h"
#include "base/fail.h"
#include "base/logging.h"

namespace mjmech {
namespace mech {

namespace {
const char* kLaunchCmd =
    "( "
    "appsrc name=h264-input max-bytes=1024000 block=false "
    "   do-timestamp=true is-live=true format=time min-latency=1 max-latency=1"
    " ! rtph264pay name=pay0 pt=96 "
    ")";
};

class RtspServer::Impl : boost::noncopyable {
 public:
  Impl(RtspServer* parent, boost::asio::io_service& service)
      : parent_(parent),
        parameters_(parent->parameters_),
        service_(service),
        parent_id_(std::this_thread::get_id()) {}

  ~Impl() {
    log_.debug("object destoyed");
  }

  void AsyncStart(base::ErrorHandler handler) {
    BOOST_ASSERT(std::this_thread::get_id() == parent_id_);

    // if gst_started is true, it means that GstReady already happened
    // and we have ignored it. Ideally, we should post gstready invocation
    // on the main glib loop instead of just crashing here.
    BOOST_ASSERT(!gst_started_);
    started_ = true;
  }

  void GstReady() {
    BOOST_ASSERT(!gst_started_);
    gst_started_ = true;
    gst_id_ = std::this_thread::get_id();
    BOOST_ASSERT(gst_id_ != parent_id_);

    if (!started_) {
      // driver disabled;
      return;
    }

    this->server_ = gst_rtsp_server_new();
    if (parameters_.port >= 0) {
      std::string service = (
          boost::format("%d") % parameters_.port).str();
      gst_rtsp_server_set_service(server_, service.c_str());
    }

    factory_ = gst_rtsp_media_factory_new();
    gst_rtsp_media_factory_set_launch(factory_, kLaunchCmd);
    // One connection for all
    gst_rtsp_media_factory_set_shared(factory_, TRUE);
    // Ignore pause requests from clients
    gst_rtsp_media_factory_set_suspend_mode(
        factory_, GST_RTSP_SUSPEND_MODE_NONE);
    // TODO theamk: set "latency"? it is 500+ sec by default
    g_signal_connect(
        factory_, "media-configure",
        G_CALLBACK(media_configure_wrapper), this);

    GstRTSPMountPoints* mounts = gst_rtsp_server_get_mount_points(server_);
    gst_rtsp_mount_points_add_factory(mounts, "/video", factory_);
    g_object_unref(mounts);

    g_signal_connect(server_, "client-connected",
                     G_CALLBACK(client_connected_wrapper), this);

    int id = gst_rtsp_server_attach(server_, NULL);
    int port = gst_rtsp_server_get_bound_port(server_);
    if (port < 0) {
      base::Fail("failed to bind RTSP server port.");
    }
    BOOST_ASSERT(id > 0);
    log_.noticeStream() <<
        boost::format("RTSP server ready at rtsp://127.0.0.1:%d/video"
                      ) % port;
  }

  void ConsumeH264Sample(GstSample* sample) {
    std::lock_guard<std::mutex> guard(appsrc_mutex_);
    if (!appsrc_h264_caps_) {
      GstCaps* caps = gst_sample_get_caps(sample);
      // Should only have one struct at this stage.
      BOOST_ASSERT(GST_CAPS_IS_SIMPLE(caps));

      if (log_.isDebugEnabled()) {
        char* caps_str = gst_caps_to_string(caps);
        log_.debug("Setting H264 caps to RTSP server: %s", caps_str);
        g_free(caps_str);
      }

      // get_caps did addref, so it all works out properly
      BOOST_ASSERT(appsrc_h264_caps_ == NULL);
      appsrc_h264_caps_ = caps;
    }

    if (!appsrc_h264_) {
      // No active session. Exit.
      return;
    }

    GstBuffer* buf = gst_sample_get_buffer(sample);
    BOOST_ASSERT(buf != NULL);
    if (appsrc_needs_iframe_ &&
        GST_BUFFER_FLAG_IS_SET(buf, GST_BUFFER_FLAG_DELTA_UNIT)) {
      // TODO theamk: add to stats
      return;
    }
    if (appsrc_needs_iframe_) {
      log_.notice("Got IFRAME in stream, now sending data");
      appsrc_needs_iframe_ = false;
    }

    if (!appsrc_offset_valid_) {
      // Record offsets from first sample.
      appsrc_offset_valid_ = true;
      if (!GST_CLOCK_TIME_IS_VALID(buf->pts)) {
        appsrc_offset_ = buf->dts;
      } else if (!GST_CLOCK_TIME_IS_VALID(buf->dts)) {
        appsrc_offset_ = buf->pts;
      } else {
        appsrc_offset_ = std::min(buf->pts, buf->dts);
      }
      appsrc_offset_dts_ = buf->dts;
      appsrc_offset_pts_ = buf->pts;
    }

    // push_buffer takes ownership, so we need to add_ref here
    buf = gst_buffer_ref(buf);

    // Make buffer writeable and apply pts/dts offsets
    buf = gst_buffer_make_writable(buf);

    if (0) {
      // One offset per value
      if (GST_CLOCK_TIME_IS_VALID(appsrc_offset_pts_)) {
        BOOST_ASSERT(GST_CLOCK_TIME_IS_VALID(buf->pts));
        buf->pts -= appsrc_offset_pts_;
      } else {
        BOOST_ASSERT(!GST_CLOCK_TIME_IS_VALID(buf->pts));
      }

      if (GST_CLOCK_TIME_IS_VALID(appsrc_offset_dts_)) {
        BOOST_ASSERT(GST_CLOCK_TIME_IS_VALID(buf->dts));
        buf->dts -= appsrc_offset_dts_;
      } else {
        BOOST_ASSERT(!GST_CLOCK_TIME_IS_VALID(buf->dts));
      }
    } else if (0) {
      // One shared offset
      if (GST_CLOCK_TIME_IS_VALID(buf->dts)) {
        buf->dts -= appsrc_offset_;
      }
      if (GST_CLOCK_TIME_IS_VALID(buf->pts)) {
        buf->pts -= appsrc_offset_;
      }
    } else {
      // Actually, camera returns completely bogus time. Just ignore it.
      buf->dts = GST_CLOCK_TIME_NONE;
      buf->pts = GST_CLOCK_TIME_NONE;
    }

    GstFlowReturn ret = gst_app_src_push_buffer(appsrc_h264_, buf);
    if (ret != GST_FLOW_OK) {
      log_.warnStream() << "Failed to push buffer to rtsp h264: " << ret;
    } else {
      samples_pushed_.fetch_add(1);
    }
  };

  void PreEmitStats(CameraDriver::CameraStats* stats) {
    std::atomic<int> other(0);
    samples_pushed_.exchange(other);
    stats->h264_frames_rtsp += other;
  };

 private:
  static void media_configure_wrapper(
      GstRTSPMediaFactory *sender, GstRTSPMedia *media, gpointer user_data) {
    static_cast<RtspServer::Impl*>(user_data)
        ->HandleMediaConfigure(sender, media);
  }

  void HandleMediaConfigure(GstRTSPMediaFactory *sender, GstRTSPMedia *media) {
    log_.notice("rtsp server got client");
    g_signal_connect(media, "unprepared",
                     G_CALLBACK(media_unprepared_wrapper), this);
    g_signal_connect(media, "new-state",
                     G_CALLBACK(media_new_state_wrapper), this);

    GstElement* main_bin = gst_rtsp_media_get_element(media);
    // Watch bus messages
    GstBus* bus = gst_element_get_bus(main_bin);
    g_signal_connect(bus, "message",
                     G_CALLBACK(media_bus_message_wrapper), this);
    gst_object_unref(GST_OBJECT(bus));

    // Find out appsink and store it
    BOOST_ASSERT(appsrc_h264_ == NULL);

    {
      std::lock_guard<std::mutex> guard(appsrc_mutex_);
      appsrc_needs_iframe_ = true;
      appsrc_offset_valid_ = false;
      appsrc_h264_ =
          GST_APP_SRC(gst_bin_get_by_name_recurse_up(
                          GST_BIN(main_bin), "h264-input"));
      if (!appsrc_h264_) {
        base::Fail("Could not find H264 appsource in RTSP pipeline");
      };
      if (appsrc_h264_caps_) {
        // Function takes a copy of caps, so original reference is still in this
        // object.
        gst_app_src_set_caps(appsrc_h264_, appsrc_h264_caps_);
      }
    };

    g_signal_connect(this->appsrc_h264_, "enough-data",
                     G_CALLBACK(appsrc_enough_data_wrapper), this);
    g_signal_connect(this->appsrc_h264_, "seek-data",
                     G_CALLBACK(appsrc_seek_data_wrapper), this);
    gst_object_unref(main_bin);
  }


  static void client_closed_wrapper(GstRTSPClient *c, gpointer user_data) {
    static_cast<RtspServer::Impl*>(user_data)->HandleClientClosed(c);
  }
  void HandleClientClosed(GstRTSPClient *client) {
    log_.noticeStream()
        << "client " << reinterpret_cast<intptr_t>(client)
        << " disconnected";
  }


  static void client_connected_wrapper(
      GstRTSPServer* s, GstRTSPClient* c,  gpointer user_data) {
    static_cast<RtspServer::Impl*>(user_data)->HandleClientConnected(s, c);
  };
  void HandleClientConnected(
      GstRTSPServer* gstrtspserver, GstRTSPClient* client) {
    GstRTSPConnection* conn = gst_rtsp_client_get_connection(client);
    log_.noticeStream()
        << "client " << reinterpret_cast<intptr_t>(client)
        << " connected from " << gst_rtsp_connection_get_ip(conn);
    g_signal_connect(client, "closed",
                     G_CALLBACK(client_closed_wrapper), this);
  }

  static void media_bus_message_wrapper(
      GstBus* bus, GstMessage* message, gpointer user_data) {
    static_cast<RtspServer::Impl*>(user_data)->
        HandleMediaBusMessage(bus, message);
  };
  void HandleMediaBusMessage(GstBus* bus, GstMessage* message) {
    switch (GST_MESSAGE_TYPE (message)) {
      case GST_MESSAGE_ERROR: {
        std::string error_text;
        GError *err = NULL;
        gchar *dbg = NULL;
        gst_message_parse_error(message, &err, &dbg);
        if (err) {
          error_text = err->message;
          g_error_free(err);
        } else {
          error_text = "unknown error";
        }
        if (dbg) {
          error_text += "\nDebug details: ";
          error_text += dbg;
          g_free(dbg);
        }
        base::Fail("RTSP Pipeline ERROR: " + error_text);
        break;
      }
        /*
          case GST_MESSAGE_STATE_CHANGED:
          case GST_MESSAGE_STREAM_STATUS:
          case GST_MESSAGE_TAG:
          case GST_MESSAGE_NEW_CLOCK: {
          // Ignore
          break;
          }
        */
      default: {
        const GstStructure* mstruct = gst_message_get_structure(message);
        char* struct_info =
            mstruct ? gst_structure_to_string(mstruct) : g_strdup("no-struct");
        log_.noticeStream()
            << boost::format("RTSP bus Message '%s' from '%s': %s")
            % GST_MESSAGE_TYPE_NAME(message)
            % GST_MESSAGE_SRC_NAME(message)
            % struct_info;
        g_free(struct_info);
        break;
      }
    }
  }

  static void media_unprepared_wrapper(GstRTSPMedia *m, gpointer user_data) {
    static_cast<RtspServer::Impl*>(user_data)->HandleMediaUnprepared(m);
  }
  void HandleMediaUnprepared(GstRTSPMedia *gstrtspmedia) {
    log_.noticeStream()
        << "Tearing down video source, there were " << error_count_
        << " errors";
    {
      std::lock_guard<std::mutex> guard(appsrc_mutex_);
      if (appsrc_h264_) {
        gst_object_unref(appsrc_h264_);
        appsrc_h264_ = NULL;
      };
    }
  }


  static void media_new_state_wrapper(
      GstRTSPMedia *gstrtspmedia, gint state, gpointer user_data) {
    // TODO theamk: if media did not go to PLAYING state after a while, raise
    // an error.
    static_cast<RtspServer::Impl*>(user_data)->
        log_.noticeStream() << "Video source in state " << state;
  }

  static void appsrc_enough_data_wrapper(GstAppSrc *src, gpointer user_data) {
    static_cast<RtspServer::Impl*>(user_data)->HandleAppsrcEnoughData(src);
  }
  void HandleAppsrcEnoughData(GstAppSrc *src) {
    static int kMaxErrors = 20;
    if (error_count_ < kMaxErrors) {
      char* name = gst_element_get_name(src);
      log_.warnStream() << "RTSP appsource '" << name
                        << "' says: enough, dropping data";;
      g_free(name);
    };
    error_count_++;
    if (error_count_ == kMaxErrors) {
      log_.warn("Further errors suppressed");
    }
  }

  static void appsrc_seek_data_wrapper(
      GstAppSrc *src, guint64 pos, gpointer user_data) {
    char* name = gst_element_get_name(src);
    static_cast<RtspServer::Impl*>(user_data)->
        log_.warnStream() << boost::format(
            "RTSP appsource '%s' says: seek me to %d. "
            "This is not suppored, trouble ahead.") % name % pos;
    g_free(name);
  }

  RtspServer* const parent_;
  const Parameters& parameters_;
  boost::asio::io_service& service_;
  const std::thread::id parent_id_;
  std::thread::id gst_id_;

  base::LogRef log_ = base::GetLogInstance("rtsp_server");

  bool started_ = false;
  bool gst_started_ = false;

  // Main server object.
  GstRTSPServer* server_ = NULL;
  // The factory for default URL
  GstRTSPMediaFactory* factory_ = NULL;;
  // Constructed media. When this is non-null, this ensures factory
  // is alive even if there are no clients.
  GstRTSPMedia* factory_media_ = NULL;;

  // Mutex for all appsrc_ variables, as they may be changed from various
  // threads (via push_h264_sample function).
  std::mutex appsrc_mutex_;

  // Current H264 appsource, or NULL if no active connection.
  // Make sure to AddRef if using outside of the mutex.
  // set to NULL when there is no active connections.
  GstAppSrc* appsrc_h264_ = NULL;
  // Caps to be applied to appsource
  GstCaps* appsrc_h264_caps_ = NULL;
  bool appsrc_needs_iframe_ = false;
  bool appsrc_offset_valid_ = false;
  GstClockTime appsrc_offset_ = 0;
  GstClockTime appsrc_offset_dts_ = 0;
  GstClockTime appsrc_offset_pts_ = 0;

  int error_count_ = 0;
  std::atomic_int samples_pushed_;
};



/// The CameraConsumer is a separate class which holds a reference to impl --
/// this way, this impl will not be destroyed while camera_driver is sending
/// frames to it.
class RtspServer::FrameConsumerImpl : public CameraFrameConsumer {
 public:
  FrameConsumerImpl(std::shared_ptr<RtspServer::Impl> impl)
      : impl_(impl) {};

  // CameraFrameConsumer interface
  virtual void GstReady() {
    impl_->GstReady();
  }

  virtual void ConsumeH264Sample(GstSample* sample) {
    impl_->ConsumeH264Sample(sample);
  }

  virtual void PreEmitStats(CameraDriver::CameraStats* stats) {
    impl_->PreEmitStats(stats);
  }

 private:
  std::shared_ptr<RtspServer::Impl> impl_;
};

RtspServer::RtspServer(boost::asio::io_service& service)
    : impl_(new Impl(this, service)),
      frame_consumer_impl_(new FrameConsumerImpl(impl_)) {
}

RtspServer::~RtspServer() {}

std::weak_ptr<CameraFrameConsumer> RtspServer::get_frame_consumer() {
  return frame_consumer_impl_;
}

void RtspServer::AsyncStart(base::ErrorHandler handler) {
  impl_->AsyncStart(handler);
}

}
}

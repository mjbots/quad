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

#include "gst_helpers.h"

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>

#include <boost/format.hpp>

#include "base/fail.h"

namespace mjmech {
namespace mech {
namespace gst {

std::string PipelineEscape(const std::string& s) {
  if (s.find(' ') == std::string::npos) {
    return s;
  }
  base::Fail("string escaping not implemented");
}

std::string FormatFraction(double val) {
  BOOST_ASSERT(val > 0);
  gint n = 0, d = 0;
  gst_util_double_to_fraction(val, &n, &d);
  return (boost::format("%d/%d") % n % d).str();
}

std::string MuxerForVideoName(const std::string& name) {
  std::string tail = name.substr(
      std::max(0, static_cast<int>(name.size()) - 4));
  if (tail == ".mkv") {
    return "matroskamux streamable=true";
  } else if (tail == ".avi") {
    return "avimux";
  } else if (tail == ".mp4") {
    return "mp4mux";
  } else {
    base::Fail(
        std::string("Unknown h264 savefile extension ") + tail);
  }
}

PipelineWrapper::PipelineWrapper(
    GstMainLoopRef& loop_ref,
    const std::string& log_prefix,
    const std::string& launch_cmd)
    : gst_loop_(loop_ref),
      log_(base::GetLogInstance(log_prefix + ".pl")),
      bus_log_(base::GetLogInstance(log_prefix + ".bus")) {
  loop_ref->quit_request_signal()->connect(
      std::bind(&PipelineWrapper::HandleShutdown, this,
                std::placeholders::_1));

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
}

void PipelineWrapper::Start() {
  // Start the pipeline
  GstStateChangeReturn rv =
      gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  if (rv == GST_STATE_CHANGE_FAILURE) {
    // no need to exit here -- we will get an error message on the bus with
    // more information.
    log_.error("Failed to start pipeline: %d", rv);
  }
}

GstElement* PipelineWrapper::GetElementByName(const char* name) {
  if (!name) {
    return GST_ELEMENT(gst_object_ref(pipeline_));
  }
  GstElement* target =
      gst_bin_get_by_name_recurse_up(GST_BIN(pipeline_), name);
  if (!target) {
    base::Fail(boost::format("Cannot find element '%s' in a pipeline")
               % name);
  }
  return target;
}

void PipelineWrapper::ConnectElementSignal(
    const char* element_name, const char* signal_name,
    GCallback callback, gpointer user_data) {

  GstElement* target = GetElementByName(element_name);
  int id = g_signal_connect(
      target, signal_name, callback, user_data);
  BOOST_ASSERT(id > 0);
  gst_object_unref(target);
}

static void identity_handoff_wrapper(
    GstElement*, GstBuffer* buffer, gpointer user_data) {
  (*static_cast<PipelineWrapper::IdentityHandoffCallback*>(user_data))
      (buffer);
}

void PipelineWrapper::ConnectIdentityHandoff(
    const char* element_name,
    const IdentityHandoffCallback& callback) {
  IdentityHandoffCallback* callback_copy =
      new IdentityHandoffCallback(callback);
  ConnectElementSignal(
      element_name, "handoff",
      G_CALLBACK(identity_handoff_wrapper), callback_copy);
}

static GstFlowReturn appsink_new_sample_wrapper(
    GstElement* sink, gpointer user_data) {

  GstSample* sample = NULL;
  g_signal_emit_by_name(sink, "pull-sample", &sample);
  if (!sample) {
    base::Fail("app sink has emitted a new-sample signal, but pull-sample"
               " failed.");
    return GST_FLOW_OK;
  }

  (*static_cast<PipelineWrapper::AppsinkNewSampleCallback*>(user_data))
      (sample);

  gst_sample_unref(sample);
  return GST_FLOW_OK;
}

void PipelineWrapper::SetupAppsink(
    const char* element_name,
    int max_buffers, bool drop,
    const AppsinkNewSampleCallback& callback) {

  GstAppSink* sink = GST_APP_SINK(GetElementByName(element_name));
  BOOST_ASSERT(sink);  // This will fail if the element type is wrong
  gst_app_sink_set_emit_signals(sink, TRUE);
  gst_app_sink_set_max_buffers(sink, max_buffers);
  gst_app_sink_set_drop(sink, drop ? TRUE : FALSE);

  AppsinkNewSampleCallback* callback_copy =
      new AppsinkNewSampleCallback(callback);
  g_signal_connect(sink, "new-sample",
                   G_CALLBACK(appsink_new_sample_wrapper), callback_copy);
  gst_object_unref(sink);
}

PipelineWrapper::AppsrcSampleCallback
PipelineWrapper::SetupAppsrc(const char* element_name) {
  GstAppSrc* src = GST_APP_SRC(GetElementByName(element_name));
  BOOST_ASSERT(src);  // This will fail if the element type is wrong

  gst_app_src_set_stream_type(src, GST_APP_STREAM_TYPE_STREAM);
  // Do not queue more than one buffer.
  //gst_app_src_set_max_bytes(src, 1);

  //return std::bind<void(void*, int)>(
  //    &PipelineWrapper::SendAppsrcSample, this,
  //    (void*)src, std::placeholders::_1, std::placeholders::_2);
  return [=](void* data, int len) {
    SendAppsrcSample(src, data, len);
  };
}

void PipelineWrapper::SendAppsrcSample(void* obj, void* data, int len) {
  BOOST_ASSERT(len > 0);
  gpointer copy = ::g_memdup(data, len);
  BOOST_ASSERT(copy); // if not, we are out of memory
  GstBuffer* buffer = gst_buffer_new_wrapped(copy, len);
  GstAppSrc* src = GST_APP_SRC(obj);
  BOOST_ASSERT(src);
  GstFlowReturn ret = gst_app_src_push_buffer(src, buffer);
  if (ret != GST_FLOW_OK) {
    base::Fail("could not push buffer");
  }
}

void PipelineWrapper::RegisterElementMessageHandler(
    const std::string& src_name_,
    const std::string& struct_name_,
    const ElementMessageHandler& handler_) {
  element_message_handlers_.push_back(
      ElementMessageHandlerRec{
        .src_name = src_name_,
            .struct_name = struct_name_,
            .handler = handler_ });
}

void PipelineWrapper::RegisterVideoAnalyzeMessageHandler(
    const std::string& src_name,
    const VideoAnalyzeMessageHandler handler) {
  int* count = new int();
  RegisterElementMessageHandler(
      src_name, "GstVideoAnalyse",
      [=](const GstStructure* data) {
        VideoAnalyzeMessage msg;
        msg.timestamp = boost::posix_time::microsec_clock::universal_time();
        msg.frame_count = *count;
        (*count)++;
        gboolean ok = gst_structure_get(
            data,
            "luma-average", G_TYPE_DOUBLE, &msg.luma_average,
            "luma-variance", G_TYPE_DOUBLE, &msg.luma_variance,
            NULL);
        BOOST_ASSERT(ok);
        handler(msg);
        return true;
      }
                                           );
}


gboolean PipelineWrapper::handle_bus_message_wrapper(
    GstBus *bus, GstMessage *message, gpointer user_data) {
  static_cast<PipelineWrapper*>(user_data)
      ->HandleBusMessage(bus, message);
  return TRUE;
}

void PipelineWrapper::HandleBusMessage(GstBus *bus, GstMessage *message) {
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
    case GST_MESSAGE_ELEMENT: {
      const GstStructure* mstruct = gst_message_get_structure(message);
      const std::string struct_name(gst_structure_get_name(mstruct));
      for (const auto& rec: element_message_handlers_) {
        if (rec.src_name != GST_MESSAGE_SRC_NAME(message)) { continue; }
        if (rec.struct_name != struct_name) { continue; }
        if (rec.handler(mstruct)) {
          print_message = false;
          break;
        }
      }
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
}

void PipelineWrapper::HandleShutdown(GstMainLoopRefObj::QuitPostponerPtr& ptr) {
  BOOST_ASSERT(std::this_thread::get_id() == gst_loop_->thread_id());
  BOOST_ASSERT(!quit_request_);
  quit_request_ = ptr;
  // send 'end of stream' event
  gst_element_send_event(pipeline_, gst_event_new_eos());
  log_.debug("shutdown requested -- sending end-of-stream");
}



}
}
}

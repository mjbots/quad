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

#pragma once

// Helpers for programs which use gstreamer
// Not intended to be included in component's headers.

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/format.hpp>

#include <gst/gst.h>

#include "base/logging.h"
#include "base/property_tree_archive.h"

#include "gst_main_loop.h"

namespace mjmech {
namespace mech {
namespace gst {

// Escape string for gstreamer pipeline
std::string PipelineEscape(const std::string& s);

// Format floating point number as fraction (10/1)
std::string FormatFraction(double val);

// Return a muxer for given file format
std::string MuxerForVideoName(const std::string& name);

template<typename Stats>
std::string FormatStatsForLogging(Stats* stats_ptr) {
  std::ostringstream out;
  base::PropertyTreeWriteArchive arch;
  arch.Accept(stats_ptr);
  for (const auto& elem: arch.tree()) {
    if (elem.first == "timestamp") { continue; }
    if (elem.second.data() == "0") { continue; }
    if (out.tellp()) { out << " "; }
    out << elem.first << "=" << elem.second.data();
  }
  if (out.tellp() == 0) {
    out << "(no data)";
  }
  return out.str();
};

struct VideoAnalyzeMessage {
  boost::posix_time::ptime timestamp;

  int frame_count = 0;
  double luma_average = 0;
  double luma_variance = 0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(frame_count));
    a->Visit(MJ_NVP(luma_average));
    a->Visit(MJ_NVP(luma_variance));
  }

  std::string toString() const {
    return (boost::format("count=%d luma_avg=%.1f luma_var=%.1f")
            % frame_count
            % (luma_average * 100.0)
            % (luma_variance * 100.0)).str();
  }
};

class PipelineWrapper : boost::noncopyable {
 public:
  PipelineWrapper(GstMainLoopRef& loop_ref,
                  const std::string& log_prefix,
                  const std::string& launch_cmd);
  virtual ~PipelineWrapper() {};

  void Start();

  // Get pipeline element by name. Fails if no such element. Do not forget
  // to release the result!
  // If @p name is NULL, return pipeline handle itself
  GstElement* GetElementByName(const char* name);

  // Connect callback to element's signal.
  // Unfortunately, we cannot wrap an std::function in a callback, as each one
  // has a different number of arguments.
  void ConnectElementSignal(const char* element_name, const char* signal_name,
                            GCallback callback, gpointer user_data);


  // Connect to identity element's 'handoff' signal -- useful to be able
  // to collect statistics / snoop on data.
  //
  // Note: The 'identity' elements are not required -- we could have
  // achieved the same effect with pad probes. I just think that identity
  // elements are slightly easier to understand.
  typedef std::function<void(GstBuffer*)> IdentityHandoffCallback;
  void ConnectIdentityHandoff(const char* element_name,
                              const IdentityHandoffCallback&);


  typedef std::function<void(GstSample*)> AppsinkNewSampleCallback;
  void SetupAppsink(const char* element_name,
                    int max_buffers, bool drop,
                    const AppsinkNewSampleCallback& callback);

  // Calling this function will copy the memory and pass it to the specific
  // appsrc.
  // TODO theamk: re-write interface to avoid an extra copy.
  typedef std::function<void(void*, int)> AppsrcSampleCallback;
  AppsrcSampleCallback SetupAppsrc(
      const char* element_name,
      const std::string& caps);

  // Function to handle element message. Should return true if the
  // message was handled.
  typedef std::function<bool(const GstStructure* data)> ElementMessageHandler;

  // Install a handler for a given element message
  void RegisterElementMessageHandler(
      const std::string& src_name,
      const std::string& struct_name,
      const ElementMessageHandler& handler);

  typedef std::function<void (const VideoAnalyzeMessage& data)
                        > VideoAnalyzeMessageHandler;
  // Install a handler for a video analyze message
  void RegisterVideoAnalyzeMessageHandler(
      const std::string& src_name,
      const VideoAnalyzeMessageHandler handler);

 protected:
  // Main handler for the bus messages
  virtual void HandleBusMessage(GstBus *bus, GstMessage *message);

 private:
  static gboolean handle_bus_message_wrapper(
      GstBus *bus, GstMessage *message, gpointer user_data);
  void HandleShutdown(GstMainLoopRefObj::QuitPostponerPtr& ptr);
  void SendAppsrcSample(void* obj, void* data, int len);

  GstMainLoopRef gst_loop_;
  GstMainLoopRefObj::QuitPostponerPtr quit_request_;
  base::LogRef log_;
  base::LogRef bus_log_;

  struct ElementMessageHandlerRec {
    const std::string src_name;
    const std::string struct_name;
    const ElementMessageHandler handler;
  };
  std::vector<ElementMessageHandlerRec> element_message_handlers_;
  GstElement* pipeline_ = NULL;
};

} // namespace gst
}
}

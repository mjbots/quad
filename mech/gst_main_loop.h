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

#include <memory>
#include <thread>

#include <boost/asio/io_service.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/signals2/signal.hpp>

#include "base/comm.h"
#include "base/visitor.h"

namespace mjmech {
namespace mech {

class GstMainLoopRefObj : boost::noncopyable {
 public:
  // TODO theamk: add other useful functions, such as
  // callLater?

  // Wait for main loop to quit. Must be called from program's main asio
  // thread. Will Fail() if thread fails to exit after a while.
  virtual void WaitForQuit() = 0;

  class QuitPostponer : boost::noncopyable {
   public:
    virtual ~QuitPostponer() {};
  };
  typedef boost::shared_ptr<QuitPostponer> QuitPostponerPtr;

  // Normally, the main loop quits when anyone calls WaitForQuit, which should
  // happen in destructor of every gst-using component. Called in gst's main
  // thread.
  //
  // If you want to delay the quit, for example to shutdown the pipeline,
  // subscribe to this signal and hold on to the reference passed in.
  virtual boost::signals2::signal<void (QuitPostponerPtr&)
                                  >* quit_request_signal() = 0;

  // Return thread id of glib main thread.
  virtual std::thread::id& thread_id() = 0;

  virtual ~GstMainLoopRefObj() {};
};

typedef std::shared_ptr<GstMainLoopRefObj> GstMainLoopRef;

// A class which holds gstreamer/glib main thread. A singleton.
class GstMainLoop : boost::noncopyable {
 public:
  template <typename Context>
    GstMainLoop(Context& context)
    : GstMainLoop(context.service,
                   &context.telemetry_registry) {}

  template <typename TelemetryRegistry>
    GstMainLoop(boost::asio::io_service& service,
                TelemetryRegistry* telemetry_registry)
    : GstMainLoop(service) {
  }

  GstMainLoop(boost::asio::io_service&);
  ~GstMainLoop();

  void AsyncStart(base::ErrorHandler handler);

  // Gstreamer has been initialized, the main thread has been created, but the
  // loop is not running yet. The consumer can set up its internal
  // settings. Invoked from main gst thread.  A consumer can take hold of ref
  // pointer to keep gstreamer main loop alive.
  boost::signals2::signal<void(GstMainLoopRef&)>* ready_signal() {
    return &ready_signal_;
  }

  struct Parameters {
    // argv/argc to pass to gst
    std::string options;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(options));
    }
  };

  Parameters* parameters() { return &parameters_; }

 private:
  class Impl;
  class MainLoopRefImpl;

  void SignalReady();

  Parameters parameters_;
  boost::signals2::signal<void(GstMainLoopRef&)> ready_signal_;
  std::shared_ptr<Impl> impl_;
};

};


}

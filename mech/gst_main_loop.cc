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

#include <condition_variable>
#include <mutex>

#include <gst/gst.h>

#include <boost/format.hpp>

#include "base/common.h"
#include "base/fail.h"
#include "base/logging.h"

namespace mjmech {
namespace mech {

class GstMainLoop::Impl : boost::noncopyable {
 public:
  Impl(GstMainLoop* parent, boost::asio::io_service& service)
      : parent_(parent),
        service_(service),
        work_(service_),
        parent_id_(std::this_thread::get_id()) {}

  ~Impl() {
    // Do not RequestQuit here -- we expect a thread with a pipeline to do so.
    //
    // TODO mafanasyev: add yet another refcounting system to quit when
    // everyone is ready.
    WaitForQuit();
  }

  void AsyncStart(base::ErrorHandler handler) {
    BOOST_ASSERT(std::this_thread::get_id() == parent_id_);

    // Capture our parent's parameters before starting our thread.
    parameters_ = parent_->parameters_;

    std::lock_guard<std::mutex> guard(thread_mutex_);
    child_ = std::thread(std::bind(&Impl::Run, this, handler));
    service_.post(std::bind(handler, base::ErrorCode()));
  }

  void WaitForQuit() {
    const auto kShutdownTimeout = std::chrono::milliseconds(2000);
    BOOST_ASSERT(std::this_thread::get_id() == parent_id_);
    std::unique_lock<std::mutex> lock(thread_mutex_);

    if (shutdown_complete_) {
      BOOST_ASSERT(!child_.joinable());
      log_.debug("was going to wait to quit, but loop has finished already");
      return;
    }

    if (!loop_) {
      log_.debug("was going to wait for quit, but we never started");
      BOOST_ASSERT(!child_.joinable());
      return;
    }

    if (!shutdown_requested_) {
      shutdown_requested_ = true;
      log_.debug("requesting child thread shutdown");
      BOOST_ASSERT(loop_);
      // TODO theamk: this will need to use to use explicit main loop if
      // we ever have more than one glib main loops.
      g_timeout_add(1, shutdown_wrapper, this);
    }

    log_.debug("waiting for child thread to quit");

    while (!shutdown_complete_) {
      if (shutdown_var_.wait_for(lock, kShutdownTimeout)
          == std::cv_status::timeout) {
        base::Fail("timed out while waiting for gst thread to quit");
      }
    }

    if (child_.joinable()) {
      child_.join();
    }
    log_.debug("child thread has exited");
  }

  std::thread::id& thread_id() {
    BOOST_ASSERT(std::thread::id() != child_id_);
    return child_id_;
  }

  boost::signals2::signal<
    void (GstMainLoopRefObj::QuitPostponerPtr&)>* quit_request_signal() {
    return &quit_request_signal_;
  }

  void NoReferencesLeft() {
    log_.debug("No external references left");
  }

 private:
  void Run(base::ErrorHandler handler) {
    {
      std::lock_guard<std::mutex> guard(thread_mutex_);
      child_id_ = child_.get_id();
      BOOST_ASSERT(child_id_ == std::this_thread::get_id());

      GstGlobalInit();

      loop_ = g_main_loop_new(NULL, FALSE);
    }

    parent_->SignalReady();

    // Start main loop.
    g_main_loop_run(loop_);
    if (quit_requested_) {
      log_.debug("child thread is exiting");
    } else {
      base::Fail("glib loop exited unexpectedly");
    }

    {
      std::unique_lock<std::mutex> lock(thread_mutex_);
      shutdown_complete_ = true;
    }
    shutdown_var_.notify_all();
  }


  void GstGlobalInit() {
    // prepare argc/argv for getstreamer
    // (the path in argv[0] may matter. so we set it to
    // bogus, but recognizeable value)
    std::string cmdline = "/mjmech-workdir/gst_main_loop";
    if (parameters_.options != "") {
      cmdline += " " + parameters_.options;
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
    base::GetLogInstance("camera_driver").debugStream()
        << "gstreamer ready: " << version;
    g_free(version);

    // Exit on critical messages from our app
    g_log_set_fatal_mask(
        NULL,
        static_cast<GLogLevelFlags>(G_LOG_LEVEL_ERROR | G_LOG_LEVEL_CRITICAL));
    // Exit on critical messages from GStreamer
    g_log_set_fatal_mask(
        "GStreamer",
        static_cast<GLogLevelFlags>(G_LOG_LEVEL_ERROR | G_LOG_LEVEL_CRITICAL));
  }


  class QuitPostponerImpl : public GstMainLoopRefObj::QuitPostponer {
   public:
    QuitPostponerImpl(std::shared_ptr<Impl> impl)
        : impl_(impl) {};

    virtual ~QuitPostponerImpl() {
      impl_->FinishShutdownRequest();
    }

   private:
    std::shared_ptr<Impl> impl_;
  };

  static gboolean shutdown_wrapper(gpointer user_data) {
    static_cast<Impl*>(user_data)->HandleShutdownRequest();
    return FALSE;
  }

  void HandleShutdownRequest() {
    BOOST_ASSERT(child_id_ == std::this_thread::get_id());
    std::shared_ptr<Impl> impl_ptr = parent_->impl_;
    GstMainLoopRefObj::QuitPostponerPtr ptr(new QuitPostponerImpl(impl_ptr));
    quit_request_signal_(ptr);
    // Will call FinishShutdownRequest in destructor
  }

  void FinishShutdownRequest() {
    BOOST_ASSERT(child_.get_id() == std::this_thread::get_id());
    log_.debug("all quit postponers are complete");
    quit_requested_ = true;
    g_main_loop_quit(loop_);
  }

  // From both
  GstMainLoop* const parent_;
  std::mutex thread_mutex_;
  std::thread child_;
  std::thread::id child_id_;
  bool quit_requested_ = false;

  bool shutdown_complete_ = false;
  bool shutdown_requested_ = false;
  std::condition_variable shutdown_var_;

  GMainLoop* loop_ = NULL;

  boost::signals2::signal<
    void (GstMainLoopRefObj::QuitPostponerPtr&)> quit_request_signal_;

  boost::asio::io_service& service_;
  boost::asio::io_service::work work_;

  base::LogRef log_ = base::GetLogInstance("gst_main");
  Parameters parameters_;

  const std::thread::id parent_id_;
};


GstMainLoop::GstMainLoop(boost::asio::io_service& service)
  : impl_(new Impl(this, service)) {};

GstMainLoop::~GstMainLoop() {}

class GstMainLoop::MainLoopRefImpl : public GstMainLoopRefObj {
 public:
  MainLoopRefImpl(std::shared_ptr<Impl>& impl)
      : impl_(impl) {};

  virtual boost::signals2::signal<void (QuitPostponerPtr&)
                                  >* quit_request_signal() {
    return impl_->quit_request_signal();
  }

  virtual void WaitForQuit() {
    impl_->WaitForQuit();
  }

  // Return thread id of glib main thread.
  virtual std::thread::id& thread_id() {
    return impl_->thread_id();
  }

  virtual void AddPeriodicTimer(double interval,
                                std::function<void()> callback) {
    // TODO theamk: forward this function to impl, too.
    g_timeout_add(interval * 1000.0, periodic_timer_wrapper,
                  new std::function<void()>(callback));
  }

  virtual ~MainLoopRefImpl() {
    impl_->NoReferencesLeft();
  }

 private:
  static gboolean periodic_timer_wrapper(gpointer data) {
    std::function<void()>* callback =
      static_cast<std::function<void()>*>(data);
    (*callback) ();
    return TRUE;
  }

  std::shared_ptr<Impl> impl_;
};


void GstMainLoop::SignalReady() {
  GstMainLoopRef ref(new MainLoopRefImpl(impl_));
  ready_signal_(ref);
}

void GstMainLoop::AsyncStart(base::ErrorHandler handler) {
  impl_->AsyncStart(handler);
}


}
}

// Copyright 2015 Mikhail Afanasyev.  All rights reserved.
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

#include "logging.h"

#include <mutex>

#include <boost/program_options.hpp>
#include <boost/noncopyable.hpp>

#include <log4cpp/OstreamAppender.hh>
#include <log4cpp/PatternLayout.hh>

#include "telemetry_log.h"

namespace mjmech {
namespace base {

namespace {
//boost::once_flag singleton_ready = BOOST_ONCE_INIT;

class SignalAppender : public log4cpp::AppenderSkeleton {
 public:
  SignalAppender(TextLogMessageSignal* signal)
      : log4cpp::AppenderSkeleton("signal"),
        signal_(signal) {
    // We want to log everything
    setThreshold(log4cpp::Priority::DEBUG);
  }

  virtual ~SignalAppender() {}

  virtual void _append(const log4cpp::LoggingEvent& event) {
    TextLogMessage msg;
    msg.timestamp =
        boost::posix_time::ptime(boost::gregorian::date(1970,1,1)) +
        boost::posix_time::seconds(event.timeStamp.getSeconds()) +
        boost::posix_time::microseconds(event.timeStamp.getMicroSeconds());
    msg.thread = event.threadName;
    msg.priority = log4cpp::Priority::getPriorityName(event.priority);
    msg.priority_int = event.priority;
    msg.ndc = event.ndc;
    msg.category = event.categoryName;
    msg.message = event.message;

    (*signal_)(&msg);
  }

  virtual bool requiresLayout () const {
    return false;
  }

  virtual void close() {};

  virtual void setLayout(log4cpp::Layout*) {};

 private:
  TextLogMessageSignal* signal_;
};

class LoggerSetup : boost::noncopyable {
 public:
  static LoggerSetup* get() {
    static std::once_flag once;
    static LoggerSetup* singleton;
    std::call_once(once, &LoggerSetup::MakeSingleton, &singleton);
    return singleton;
  }

  void AddLoggingOptions(boost::program_options::options_description* desc) {
    namespace po = boost::program_options;

    // TODO theamk: consider always logging all messages to telemetry? but this
    // may trigger expensive debug message generation.

    // TODO theamk: consider adding flag which turns off all stderr messages,
    // only leaves messages in telemetry log.

    desc->add_options()
        ("verbose,v",
         po::bool_switch()->notifier(
             std::bind(&LoggerSetup::HandleVerbose,
                       this, std::placeholders::_1)),
         "enable debug logging for all components")
        ("trace,t",
         po::value<std::vector<std::string> >()->notifier(
             std::bind(&LoggerSetup::HandleTrace, this, std::placeholders::_1)),
         "enable debug logging for this exact source");
  }

  //void WriteLogToTelemetryLog(TelemetryLog* log) {  }

  TextLogMessageSignal* GetLogMessageSignal() {
    return &signal_;
  }

 private:
  LoggerSetup() {
    log4cpp::OstreamAppender* appender =
        new log4cpp::OstreamAppender("stderr", &std::cerr);
    appender->setThreshold(log4cpp::Priority::DEBUG);

    log4cpp::PatternLayout* layout = new log4cpp::PatternLayout();
    layout->setConversionPattern("%d{%H:%M:%S.%l} %p[%c]%x: %m%n");
    appender->setLayout(layout);

    log4cpp::Category& root = log4cpp::Category::getRoot();
    root.setPriority(log4cpp::Priority::DEBUG);
    // passes ownership
    root.addAppender(appender);

    SignalAppender* appender2 = new SignalAppender(&signal_);
    // passes ownership
    root.addAppender(appender2);
  };

  static void MakeSingleton(LoggerSetup** out) {
    *out = new LoggerSetup();
  }

  void HandleVerbose(bool value) {
    log4cpp::Category::getRoot().setPriority(
        value ? log4cpp::Priority::DEBUG : log4cpp::Priority::NOTICE);
  }

  void HandleTrace(const std::vector<std::string> value) {
    for (auto& s: value) {
      if (s.substr(0, 1) == "~") {
        log4cpp::Category::getInstance(s.substr(1)).setPriority(
            log4cpp::Priority::WARN);
      } else {
        log4cpp::Category::getInstance(s).setPriority(
            log4cpp::Priority::DEBUG);
      }
    }
  }

  TextLogMessageSignal signal_;
};


}

void AddLoggingOptions(boost::program_options::options_description* desc) {
  LoggerSetup::get()->AddLoggingOptions(desc);
}

void InitLogging() {
  LoggerSetup::get();
}

//void WriteLogToTelemetryLog(TelemetryLog* log) {
//  LoggerSetup::get()->WriteLogToTelemetryLog(log);
//}

TextLogMessageSignal* GetLogMessageSignal() {
  return LoggerSetup::get()->GetLogMessageSignal();
}

LogRef GetLogInstance(const std::string& name) {
  LoggerSetup::get();
  return log4cpp::Category::getInstance(name);
}

LogRef GetSubLogger(LogRef parent, const std::string& name) {
  // Somehow log4cpp does not export this method
  // (depite having logging hierarchy)
  return GetLogInstance(parent.getName() + "." + name);
}



}
}

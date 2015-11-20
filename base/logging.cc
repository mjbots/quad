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

namespace mjmech {
namespace base {

namespace {
//boost::once_flag singleton_ready = BOOST_ONCE_INIT;

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
    root.addAppender(appender);
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
};


}

void AddLoggingOptions(boost::program_options::options_description* desc) {
  LoggerSetup::get()->AddLoggingOptions(desc);
}

void InitLogging() {
  LoggerSetup::get();
}

LogRef GetLogInstance(const std::string& name) {
  LoggerSetup::get();
  return log4cpp::Category::getInstance(name);
}



}
}

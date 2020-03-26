// Copyright 2015-2020 Josh Pieper, jjp@pobox.com.
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

#include <boost/signals2/signal.hpp>

#include "mjlib/telemetry/binary_write_archive.h"
#include "mjlib/telemetry/file_writer.h"


namespace mjmech {
namespace base {
/// A registrar which emits every instance of every record to a
/// TelemetryLog instance using the TelemetryArchive for
/// serialization.
///
/// NOTE: Ideally this would be noncopyable, but std::tuple doesn't
/// currently allow construction of noncopyable members.
///
/// NOTE: In the future, this could have policies around which records
/// are written to the log and at what rate.
class TelemetryLogRegistrar {
 public:
  TelemetryLogRegistrar(mjlib::telemetry::FileWriter* telemetry_log)
    : telemetry_log_(telemetry_log) {}

  template <typename T>
  void Register(const std::string& name,
                boost::signals2::signal<void (const T*)>* signal) {
    const auto identifier = telemetry_log_->AllocateIdentifier(name);
    telemetry_log_->WriteSchema(
        identifier,
        mjlib::telemetry::BinarySchemaArchive::template schema<T>());
    signal->connect(std::bind(&TelemetryLogRegistrar::HandleData<T>,
                              this, identifier,
                              std::placeholders::_1));
  }

  template <typename T>
  void HandleData(mjlib::telemetry::FileWriter::Identifier identifier,
                  const T* data) {
    // If the log isn't open, don't even bother serializing things.
    if (!telemetry_log_->IsOpen()) { return; }

    auto buffer = telemetry_log_->GetBuffer();
    mjlib::telemetry::BinaryWriteArchive(*buffer).Accept(data);
    telemetry_log_->WriteData({}, identifier, std::move(buffer));
  }

  mjlib::telemetry::FileWriter* const telemetry_log_;
};
}
}

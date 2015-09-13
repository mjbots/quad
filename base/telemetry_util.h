// Copyright 2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/iostreams/stream.hpp>

#include "recording_stream.h"
#include "telemetry_format.h"

namespace mjmech {
namespace base {

/// Produce a textual representation of a telemetry schema and
/// optionally a data image.
class TelemetrySchemaReader {
 public:
  typedef TelemetryFormat TF;

  TelemetrySchemaReader(TelemetryReadStream<>& stream,
                        TelemetryReadStream<>* data,
                        std::ostream& out)
      : stream_(stream),
        data_(data),
        out_(out) {}

  void Read() {
    uint32_t flags = stream_.Read<uint32_t>();
    out_ << boost::format("[SchemaFlags]: %08X;\n") % flags;

    ReadSchemaObject(0, kAllInfo);
    out_ << "\n";
  }

 private:
  enum Semicolon {
    kSemicolon,
    kSkipSemicolon,
  };

  enum DataPolicy {
    kAllInfo,
    kNamesAndData,
    kTypesOnly,
    kDataOnly,
  };

  static bool DisplayNames(DataPolicy p) {
    switch (p) {
      case kAllInfo: { return true; }
      case kNamesAndData: { return true; }
      case kTypesOnly: { return true; }
      case kDataOnly: { return false; }
    }
    return false;
  }

  static bool DisplayTypes(DataPolicy p) {
    switch (p) {
      case kAllInfo: { return true; }
      case kNamesAndData: { return false; }
      case kTypesOnly: { return true; }
      case kDataOnly: { return false; }
    }
    return false;
  }

  static bool DisplayData(DataPolicy p) {
    switch (p) {
      case kAllInfo: { return true; }
      case kNamesAndData: { return true; }
      case kTypesOnly: { return false; }
      case kDataOnly: { return true; }
    }
    return false;
  }

  void DoPrimitiveData(TF::FieldType type) {
    BOOST_ASSERT(data_);

    typedef TF::FieldType FT;

    switch (type) {
      case FT::kBool: { out_ << data_->Read<bool>(); break; }
      case FT::kInt8: {
        out_ << static_cast<int>(data_->Read<int8_t>());
        break;
      }
      case FT::kUInt8: {
        out_ << static_cast<int>(data_->Read<uint8_t>());
        break;
      }
      case FT::kInt16: { out_ << data_->Read<int16_t>(); break; }
      case FT::kUInt16: { out_ << data_->Read<uint16_t>(); break; }
      case FT::kInt32: { out_ << data_->Read<int32_t>(); break; }
      case FT::kUInt32: { out_ << data_->Read<uint32_t>(); break; }
      case FT::kInt64: { out_ << data_->Read<int64_t>(); break; }
      case FT::kUInt64: { out_ << data_->Read<uint64_t>(); break; }
      case FT::kFloat32: { out_ << data_->Read<float>(); break; }
      case FT::kFloat64: { out_ << data_->Read<double>(); break; }
      case FT::kString: {
        out_ << "\"" << data_->ReadString() << "\"";
        break;
      }
      case FT::kPtime: {
        out_ << data_->ReadPtime();
        break;
      }
      case FT::kFinal:
      case FT::kPair:
      case FT::kArray:
      case FT::kVector:
      case FT::kObject:
      case FT::kOptional:
      case FT::kEnum: {
        BOOST_ASSERT(false);
      }
    }
  }

  void RenderVariableSize(int indent, uint32_t num_elements,
                          DataPolicy data_policy) {
    RecordingInputSource ris(stream_.stream());
    boost::iostreams::stream<
      RecordingInputSource> io_stream(ris, 1, 1);
    TelemetryReadStream<> recording_stream(io_stream);
    TelemetrySchemaReader sub_reader(recording_stream, nullptr, out_);
    sub_reader.DoField(indent + 2, 0, "", kSkipSemicolon, data_policy);

    if (DisplayData(data_policy) && data_) {
      if (DisplayNames(data_policy)) {
        out_ << " = ";
      }
      out_ << "[";
      for (uint32_t i = 0; i < num_elements; i++) {
        std::istringstream istr(io_stream->str());
        TelemetryReadStream<> sub_stream(istr);
        TelemetrySchemaReader field_reader(sub_stream, data_, out_);
        field_reader.DoField(indent + 2, 0, "", kSkipSemicolon, kDataOnly);
        if ((i + 1) != num_elements) { out_ << ","; }
      }
      out_ << "]";
    }
  }

  bool DoField(int indent,
               uint32_t field_flags, const std::string& field_name,
               Semicolon final_semicolon,
               DataPolicy data_policy) {
    std::string instr = std::string(indent, ' ');

    TF::FieldType field_type = static_cast<TF::FieldType>(
        stream_.Read<uint32_t>());

    if (field_type == TF::FieldType::kFinal) {
      return true;
    }

    std::string flags_str;
    if (field_flags) {
      flags_str = (boost::format(" f=%08X") % field_flags).str();
    }
    if (DisplayNames(data_policy)) {
      out_ << boost::format("%s%s") % instr % field_name;
    }

    if (DisplayTypes(data_policy)) {
      out_ << boost::format(": %s%s") % FieldTypeStr(field_type) % flags_str;
    }

    switch (field_type) {
      case TF::FieldType::kFinal: {
        BOOST_ASSERT(false);
      }
      case TF::FieldType::kBool:
      case TF::FieldType::kInt8:
      case TF::FieldType::kUInt8:
      case TF::FieldType::kInt16:
      case TF::FieldType::kUInt16:
      case TF::FieldType::kInt32:
      case TF::FieldType::kUInt32:
      case TF::FieldType::kInt64:
      case TF::FieldType::kUInt64:
      case TF::FieldType::kFloat32:
      case TF::FieldType::kFloat64:
      case TF::FieldType::kPtime:
      case TF::FieldType::kString: {
        if (data_) {
          if (DisplayNames(data_policy)) {
            out_ << " = ";
          }
          if (DisplayData(data_policy)) {
            DoPrimitiveData(field_type);
          }
        }
        break;
      }
      case TF::FieldType::kPair: {
        out_ << "<\n";
        DoField(indent + 2, 0, "", kSkipSemicolon, data_policy);
        out_ << "\n";
        DoField(indent + 2, 0, "", kSkipSemicolon, data_policy);
        out_ << "\n" << instr << "  >";
        break;
      }
      case TF::FieldType::kArray: {
        uint32_t nelements = stream_.Read<uint32_t>();
        if (nelements >
            static_cast<uint32_t>(TF::BlockOffsets::kMaxBlockSize)) {
          throw std::runtime_error("corrupt array size");
        }
        out_ << boost::format("[%d]\n") % nelements;

        RenderVariableSize(indent, nelements, data_policy);

        break;
      }
      case TF::FieldType::kVector: {
        if (DisplayNames(data_policy)) {
          out_ << "\n";
        }

        uint32_t nelements = 0;
        if (data_) {
          nelements = data_->Read<uint32_t>();
        }

        RenderVariableSize(indent, nelements, data_policy);

        break;
      }
      case TF::FieldType::kObject: {
        out_ << "\n";
        ReadSchemaObject(indent + 2, data_policy);
        break;
      }
      case TF::FieldType::kOptional: {
        if (DisplayNames(data_policy)) {
          out_ << "\n";
        }

        uint8_t present = 0;
        if (data_) {
          present = data_->Read<uint8_t>();
        }

        RenderVariableSize(indent, present, data_policy);
        break;
      }
      default: {
        BOOST_ASSERT(false);
      }
    }

    if (data_policy == kAllInfo) {
      switch (final_semicolon) {
        case kSemicolon: { out_ << ";\n"; break; }
        case kSkipSemicolon: { break; }
      }
    }
    return false;
  }

  void ReadSchemaObject(int indent_in, DataPolicy data_policy) {
    out_ << boost::format("%s{") % std::string(indent_in, ' ');
    if (DisplayNames(data_policy)) { out_ << "\n"; }

    int indent = indent_in + 2;
    std::string instr = std::string(indent, ' ');
    uint32_t flags = stream_.Read<uint32_t>();
    if (data_policy != kDataOnly && flags != 0) {
      out_ << boost::format("%s[Flags]: %08X;\n") %
          instr %
          flags;
    }
    bool done = false;
    while (!done) {
      uint32_t field_flags = stream_.Read<uint32_t>();
      std::string field_name = stream_.ReadString();

      const bool final = DoField(
          indent, field_flags, field_name, kSemicolon, data_policy);
      if (final) { done = true; }
    }

    if (DisplayNames(data_policy)) {
      out_ << boost::format("%s") % std::string(indent_in, ' ');
    }
    out_ << "}";
  }

  static std::string FieldTypeStr(TF::FieldType v) {
    switch (v) {
      case TF::FieldType::kFinal: { return "kFinal"; }
      case TF::FieldType::kBool: { return "kBool"; }
      case TF::FieldType::kInt8: { return "kInt8"; }
      case TF::FieldType::kUInt8: { return "kUInt8"; }
      case TF::FieldType::kInt16: { return "kInt16"; }
      case TF::FieldType::kUInt16: { return "kUInt16"; }
      case TF::FieldType::kInt32: { return "kInt32"; }
      case TF::FieldType::kUInt32: { return "kUInt32"; }
      case TF::FieldType::kInt64: { return "kInt64"; }
      case TF::FieldType::kUInt64: { return "kUInt64"; }
      case TF::FieldType::kFloat32: { return "kFloat32"; }
      case TF::FieldType::kFloat64: { return "kFloat64"; }
      case TF::FieldType::kPtime: { return "kPtime"; }
      case TF::FieldType::kString: { return "kString"; }
      case TF::FieldType::kPair: { return "kPair"; }
      case TF::FieldType::kArray: { return "kArray"; }
      case TF::FieldType::kVector: { return "kVector"; }
      case TF::FieldType::kObject: { return "kObject"; }
      case TF::FieldType::kOptional: { return "kOptional"; }
      default: {
        BOOST_ASSERT(false);
      }
    }
  }

  TelemetryReadStream<>& stream_;
  TelemetryReadStream<>* const data_;
  std::ostream& out_;
};

}
}

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

#include <type_traits>

#include <boost/assert.hpp>

#include "fast_stream.h"
#include "telemetry_format.h"
#include "visit_archive.h"

namespace mjmech {
namespace base {
/// Generate schema and binary records according to the structure
/// defined in telemetry_format.h.
template <typename RootSerializable>
class TelemetryWriteArchive {
 public:
  typedef TelemetryFormat TF;

  TelemetryWriteArchive() {}

  static std::string schema() { return MakeSchema(); }

  template <typename OStream>
  static void Serialize(const RootSerializable* serializable,
                        OStream& stream_in) {
    TelemetryWriteStream<OStream> stream(stream_in);

    DataVisitor<TelemetryWriteStream<OStream> > visitor(stream);
    visitor.Accept(const_cast<RootSerializable*>(serializable));
  }

  static std::string Serialize(const RootSerializable* serializable) {
    FastOStringStream ostr;
    Serialize(serializable, ostr);
    return ostr.str();
  }

  static std::string MakeSchema() {
    FastOStringStream ostr;
    TelemetryWriteStream<FastOStringStream> stream(ostr);

    stream.Write(static_cast<uint32_t>(0)); // SchemaFlags

    WriteSchemaObject(stream, static_cast<RootSerializable*>(0));

    return ostr.str();
  }

 private:
  template <typename WriteStream,
            typename Serializable>
  static void WriteSchemaObject(WriteStream& stream,
                                Serializable* serializable) {
    SchemaVisitor<WriteStream> visitor(stream);
    visitor.Accept(serializable);

    visitor.Finish();
  }

  template <typename T>
  struct FakeNvp {
    FakeNvp(T* value) : value_(value) {}
    const T& get_value() const { return *value_; }
    T* value() const { return value_; }

    T* const value_;
  };

  template <typename WriteStream>
  class SchemaVisitor : public VisitArchive<SchemaVisitor<WriteStream>> {
   public:
    SchemaVisitor(WriteStream& stream) : stream_(stream) {
      stream.Write(static_cast<uint32_t>(0)); // ObjectFlags
    }

    void Finish() {
      // Write out the "final" record.
      stream_.Write(static_cast<uint32_t>(0));
      stream_.Write(std::string());
      stream_.Write(static_cast<uint32_t>(TF::FieldType::kFinal));
    }

    template <typename NameValuePair>
    void Visit(const NameValuePair& pair) {
      stream_.Write(static_cast<uint32_t>(0)); // FieldFlags;
      stream_.Write(std::string(pair.name()));

      VisitArchive<SchemaVisitor>::Visit(pair);
    }

    template <typename NameValuePair>
    void VisitSerializable(const NameValuePair& pair) {
      stream_.Write(static_cast<uint32_t>(TF::FieldType::kObject));

      SchemaVisitor visitor(stream_);
      visitor.Accept(pair.value());
      visitor.Finish();
    }

    template <typename NameValuePair>
    void VisitEnumeration(const NameValuePair& pair) {
      stream_.Write(static_cast<uint32_t>(TF::FieldType::kEnum));

      FastOStringStream enum_ostr;
      TelemetryWriteStream<FastOStringStream> enum_stream(enum_ostr);

      const auto items = pair.enumeration_mapper();
      uint32_t nvalues = items.size();
      enum_stream.Write(nvalues);

      for (const auto& pair: items) {
        uint32_t key = pair.first;
        std::string value = pair.second;
        enum_stream.Write(key);
        enum_stream.Write(value);
      }

      stream_.Write(static_cast<uint32_t>(enum_ostr.data()->size()));
      stream_.RawWrite(enum_ostr.data()->data(),
                       enum_ostr.data()->size());
    }

    template <typename NameValuePair>
    void VisitScalar(const NameValuePair& pair) {
      VisitHelper(pair, pair.value(), 0);
    }

   private:
    template <typename NameValuePair, typename First, typename Second>
    void VisitHelper(const NameValuePair& pair,
                     std::pair<First, Second>*,
                     int) {
      stream_.Write(static_cast<uint32_t>(TF::FieldType::kPair));

      VisitArchive<SchemaVisitor>::Visit(
          FakeNvp<First>(&pair.value()->first));
      VisitArchive<SchemaVisitor>::Visit(
          FakeNvp<Second>(&pair.value()->second));
    }

    template <typename NameValuePair, typename T, std::size_t N>
    void VisitHelper(const NameValuePair& pair,
                     std::array<T, N>*,
                     int) {
      stream_.Write(static_cast<uint32_t>(TF::FieldType::kArray));
      stream_.Write(static_cast<uint32_t>(N));

      VisitArchive<SchemaVisitor>::Visit(
          FakeNvp<T>(static_cast<T*>(nullptr)));
    }

    template <typename NameValuePair, typename T>
    void VisitHelper(const NameValuePair& pair,
                     std::vector<T>*,
                     int) {
      stream_.Write(static_cast<uint32_t>(TF::FieldType::kVector));

      VisitArchive<SchemaVisitor>::Visit(
          FakeNvp<T>(static_cast<T*>(nullptr)));
    }

    template <typename NameValuePair, typename T>
    void VisitHelper(const NameValuePair& pair,
                     boost::optional<T>*,
                     int) {
      stream_.Write(static_cast<uint32_t>(TF::FieldType::kOptional));

      VisitArchive<SchemaVisitor>::Visit(FakeNvp<T>(static_cast<T*>(0)));
    }

    template <typename NameValuePair, typename T>
    void VisitHelper(const NameValuePair& pair,
                     T*,
                     long) {
      VisitPrimitive(pair);
    }

    template <typename NameValuePair>
    void VisitPrimitive(const NameValuePair& pair) {
      stream_.Write(static_cast<uint32_t>(FindType(pair.value())));
    }


    WriteStream& stream_;
  };

  template <typename WriteStream>
  class DataVisitor : public VisitArchive<DataVisitor<WriteStream>> {
   public:
    DataVisitor(WriteStream& stream) : stream_(stream) {}

    template <typename NameValuePair>
    void VisitSerializable(const NameValuePair& pair) {
      DataVisitor sub(stream_);
      sub.Accept(pair.value());
    }

    template <typename NameValuePair>
    void VisitScalar(const NameValuePair& pair) {
      typename std::decay<decltype(pair.get_value())>::type * dummy = nullptr;
      VisitHelper(pair, dummy, 0);
    }

   private:
    template <typename NameValuePair, typename First, typename Second>
    void VisitHelper(const NameValuePair& pair,
                     std::pair<First, Second>*,
                     int) {
      VisitArchive<DataVisitor>::Visit(
          FakeNvp<First>(&pair.value()->first));
      VisitArchive<DataVisitor>::Visit(
          FakeNvp<Second>(&pair.value()->second));
    }

    template <typename NameValuePair, typename T, std::size_t N>
    void VisitHelper(const NameValuePair& pair,
                     std::array<T, N>*,
                     int) {
      auto value = pair.value();
      for (int i = 0; i < N; i++) {
        VisitArchive<DataVisitor>::Visit(
            FakeNvp<T>(&(*value)[i]));
      }
    }

    template <typename NameValuePair, typename T>
    void VisitHelper(const NameValuePair& pair,
                     std::vector<T>*,
                     int) {
      auto value = pair.value();
      stream_.Write(static_cast<uint32_t>(value->size()));
      for (int i = 0; i < static_cast<int>(value->size()); i++) {
        VisitArchive<DataVisitor>::Visit(
            FakeNvp<T>(&(*value)[i]));
      }
    }

    template <typename NameValuePair, typename T>
    void VisitHelper(const NameValuePair& pair,
                     boost::optional<T>*,
                     int) {
      auto value = pair.value();
      stream_.Write(static_cast<uint8_t>(*value ? 1 : 0));
      if (*value) {
        VisitArchive<DataVisitor>::Visit(
            FakeNvp<T>(&(**value)));
      }
    }

    template <typename NameValuePair, typename T>
    void VisitHelper(const NameValuePair& pair,
                     T*,
                     long) {
      VisitPrimitive(pair);
    }

    template <typename NameValuePair>
    void VisitPrimitive(const NameValuePair& pair) {
      stream_.Write(pair.get_value());
    }

    WriteStream& stream_;
  };

  static TF::FieldType FindType(bool*) { return TF::FieldType::kBool; }
  static TF::FieldType FindType(int8_t*) { return TF::FieldType::kInt8; }
  static TF::FieldType FindType(uint8_t*) { return TF::FieldType::kUInt8; }
  static TF::FieldType FindType(int16_t*) { return TF::FieldType::kInt16; }
  static TF::FieldType FindType(uint16_t*) { return TF::FieldType::kUInt16; }
  static TF::FieldType FindType(int32_t*) { return TF::FieldType::kInt32; }
  static TF::FieldType FindType(uint32_t*) { return TF::FieldType::kUInt32; }
  static TF::FieldType FindType(int64_t*) { return TF::FieldType::kInt64; }
  static TF::FieldType FindType(uint64_t*) { return TF::FieldType::kUInt64; }
  static TF::FieldType FindType(float*) { return TF::FieldType::kFloat32; }
  static TF::FieldType FindType(double*) { return TF::FieldType::kFloat64; }
  static TF::FieldType FindType(std::string*) { return TF::FieldType::kString; }
  static TF::FieldType FindType(boost::posix_time::ptime*) {
    return TF::FieldType::kPtime;
  }
};
}
}

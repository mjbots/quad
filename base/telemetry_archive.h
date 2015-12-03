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

#ifndef MJMECH_DISABLE_BOOST
#include <boost/optional.hpp>
#endif

#include "fast_stream.h"
#include "telemetry_archive_detail.h"
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

  template <typename OStream>
  static void WriteSchema(OStream& stream_in) {
    TelemetryWriteStream<OStream> stream(stream_in);
    stream.Write(static_cast<uint32_t>(0)); // SchemaFlags

    WriteSchemaObject(stream, static_cast<RootSerializable*>(0));
  }

  static std::string MakeSchema() {
    FastOStringStream ostr;
    WriteSchema(ostr);
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

  template <typename WriteStream>
  class SchemaVisitor : public VisitArchive<SchemaVisitor<WriteStream>> {
   public:
    SchemaVisitor(WriteStream& stream) : stream_(stream) {
      stream.Write(static_cast<uint32_t>(0)); // ObjectFlags
    }

    void Finish() {
      // Write out the "final" record.
      stream_.Write(static_cast<uint32_t>(0));
      stream_.Write(gsl::ensure_z(""));
      stream_.Write(static_cast<uint32_t>(TF::FieldType::kFinal));
    }

    template <typename NameValuePair>
    void Visit(const NameValuePair& pair) {
      stream_.Write(static_cast<uint32_t>(0)); // FieldFlags;
      stream_.Write(gsl::ensure_z(pair.name()));

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

      const auto items = pair.enumeration_mapper();
      uint32_t nvalues = items.size();
      stream_.Write(nvalues);

      for (const auto& pair: items) {
        uint32_t key = static_cast<uint32_t>(pair.first);
        stream_.Write(key);
        stream_.Write(gsl::ensure_z(pair.second));
      }
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
          detail::FakeNvp<First>(&pair.value()->first));
      VisitArchive<SchemaVisitor>::Visit(
          detail::FakeNvp<Second>(&pair.value()->second));
    }

    template <typename NameValuePair, typename T, std::size_t N>
    void VisitHelper(const NameValuePair& pair,
                     std::array<T, N>*,
                     int) {
      stream_.Write(static_cast<uint32_t>(TF::FieldType::kArray));
      stream_.Write(static_cast<uint32_t>(N));

      VisitArchive<SchemaVisitor>::Visit(
          detail::FakeNvp<T>(static_cast<T*>(nullptr)));
    }

    template <typename NameValuePair, typename T>
    void VisitHelper(const NameValuePair& pair,
                     std::vector<T>*,
                     int) {
      stream_.Write(static_cast<uint32_t>(TF::FieldType::kVector));

      VisitArchive<SchemaVisitor>::Visit(
          detail::FakeNvp<T>(static_cast<T*>(nullptr)));
    }

#ifndef MJMECH_DISABLE_BOOST
    template <typename NameValuePair, typename T>
    void VisitHelper(const NameValuePair& pair,
                     boost::optional<T>*,
                     int) {
      stream_.Write(static_cast<uint32_t>(TF::FieldType::kOptional));

      VisitArchive<SchemaVisitor>::Visit(
          detail::FakeNvp<T>(static_cast<T*>(0)));
    }
#endif

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
  class DataVisitor : public detail::DataVisitorBase<
      DataVisitor<WriteStream>, WriteStream> {
   public:
    typedef detail::DataVisitorBase<DataVisitor<WriteStream>, WriteStream> Base;
    DataVisitor(WriteStream& stream) : Base(stream) {}

    template <typename NameValuePair>
    void VisitVector(const NameValuePair& pair) {
      auto value = pair.value();
      this->stream_.Write(static_cast<uint32_t>(value->size()));
      for (int i = 0; i < static_cast<int>(value->size()); i++) {
        Base::Visit(detail::MakeFakeNvp(&(*value)[i]));
      }
    }

    template <typename NameValuePair>
    void VisitOptional(const NameValuePair& pair) {
      auto value = pair.value();
      this->stream_.Write(static_cast<uint8_t>(*value ? 1 : 0));
      if (*value) {
        Base::Visit(detail::MakeFakeNvp(&(**value)));
      }
    }

    template <typename NameValuePair>
    void VisitPrimitive(const NameValuePair& pair) {
      this->stream_.Write(pair.get_value());
    }

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
#ifndef MJMECH_DISABLE_BOOST
  static TF::FieldType FindType(boost::posix_time::ptime*) {
    return TF::FieldType::kPtime;
  }
#endif
};

/// This archive can read a serialized structure assuming that the
/// schema exactly matches what was serialized.  An out of band
/// mechanism is required to enforce this.
template <typename RootSerializable>
class TelemetrySimpleReadArchive {
 public:
  typedef TelemetryFormat TF;

  template <typename IStream>
  static void Deserialize(RootSerializable* serializable,
                          IStream& stream_in) {
    TelemetryReadStream<IStream> stream(stream_in);

    DataVisitor<TelemetryReadStream<IStream> > visitor(stream);
    visitor.Accept(serializable);
  }

 private:
  template <typename ReadStream>
  class DataVisitor : public detail::DataVisitorBase<
      DataVisitor<ReadStream>, ReadStream> {
   public:
    typedef detail::DataVisitorBase<DataVisitor<ReadStream>, ReadStream> Base;
    DataVisitor(ReadStream& stream) : Base(stream) {}

    template <typename NameValuePair>
    void VisitVector(const NameValuePair& pair) {
      auto value = pair.value();
      uint32_t size = this->stream_.template Read<uint32_t>();
      value->resize(size);
      for (int i = 0; i < static_cast<int>(value->size()); i++) {
        Base::Visit(detail::MakeFakeNvp(&(*value)[i]));
      }
    }

#ifndef MJMECH_DISABLE_BOOST
    template <typename NameValuePair>
    void VisitOptional(const NameValuePair& pair) {
      auto value = pair.value();
      uint8_t present = this->stream_.template Read<uint8_t>();
      if (!present) {
        *value = boost::none;
      } else {
        typedef typename std::decay<decltype(pair.get_value())>::type ValueType;
        *value = ValueType();
        Base::Visit(detail::MakeFakeNvp(&(**value)));
      }
    }
#endif

    template <typename NameValuePair>
    void VisitPrimitive(const NameValuePair& pair) {
      typedef typename std::decay<decltype(pair.get_value())>::type T;
      T* t = nullptr;
      ReadPrimitive(pair, t, 0);
    }

    template <typename NameValuePair>
    void ReadPrimitive(const NameValuePair& pair, std::string*, int) {
      pair.set_value(this->stream_.ReadString());
    }

#ifndef MJMECH_DISABLE_BOOST
    template <typename NameValuePair>
    void ReadPrimitive(const NameValuePair& pair,
                       boost::posix_time::ptime*, int) {
      pair.set_value(this->stream_.ReadPtime());
    }
#endif

    template <typename NameValuePair, typename T>
    void ReadPrimitive(const NameValuePair& pair,
                       T* value, long) {
      pair.set_value(this->stream_.template Read<T>());
    }

  };
};
}
}

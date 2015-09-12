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

#include <fstream>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/program_options.hpp>

#include "telemetry_util.h"

namespace {
namespace po = boost::program_options;
using namespace legtool;

struct Options {
  bool schema = false;
  std::vector<std::string> inputs;
};

void DoSchema(const Options& options) {
  std::string schema_filename;
  std::string binary_filename;
  for (auto name: options.inputs) {
    if (boost::ends_with(name, ".schema")) { schema_filename = name; }
    if (boost::ends_with(name, ".bin")) { binary_filename = name; }
  }

  if (schema_filename.empty()) {
    throw std::runtime_error("No .schema file specified");
  }

  std::ifstream schema(schema_filename);
  if (!schema.is_open()) {
    throw std::runtime_error("could not open schema: " + schema_filename);
  }
  TelemetryReadStream<> stream(schema);

  std::unique_ptr<std::ifstream> binary;
  std::unique_ptr<TelemetryReadStream<>> binary_stream;
  if (!binary_filename.empty()) {
    binary.reset(new std::ifstream(binary_filename));
    if (!binary->is_open()) {
      throw std::runtime_error("could not open binary: " + binary_filename);
    }
    binary_stream.reset(new TelemetryReadStream<>(*binary));
  }

  TelemetrySchemaReader(stream, binary_stream.get(), std::cout).Read();
}

int work(int argc, char** argv) {
  Options options;

  po::options_description desc("Allowable options");
  desc.add_options()
      ("help,h", "display usage message")
      ("input,i", po::value(&options.inputs), "input files")
      ("schema", po::bool_switch(&options.schema), "read schema and/or binary")
      ;

  po::positional_options_description pos;
  pos.add("input", -1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).
            options(desc).positional(pos).run(), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cerr << desc;
    return 0;
  }

  if (options.schema) {
    DoSchema(options);
  }

  return 0;
}
}

int main(int argc, char** argv) {
  try {
    return work(argc, argv);
  } catch (std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}

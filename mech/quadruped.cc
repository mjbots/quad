// Copyright 2019-2020 Josh Pieper, jjp@pobox.com.
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

#include "mech/quadruped.h"

#include "base/logging.h"
#include "mech/quadruped_debug.h"

namespace pl = std::placeholders;

namespace mjmech {
namespace mech {

class Quadruped::Impl {
 public:
  Impl(base::Context& context)
      : executor_(context.executor),
        factory_(context.factory.get()) {
    m_.multiplex_client = std::make_unique<MultiplexClient>(executor_);
    m_.quadruped_control = std::make_unique<QuadrupedControl>(context);
    m_.web_control = std::make_unique<WebControl>(
        context.executor, m_.quadruped_control.get());

    m_.multiplex_client->RequestClient([this](const auto& ec, auto* client) {
        mjlib::base::FailIf(ec);
        m_.quadruped_control->SetClient(client);
      });

    debug_stream_.type = mjlib::io::StreamFactory::Type::kTcpServer;
    debug_stream_.tcp_server_port = 4556;
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    base::StartArchive::Start(
        &m_, [this, callback=std::move(callback)](auto ec) mutable {
          mjlib::base::FailIf(ec);
          this->StartDebug(std::move(callback));
        });
  }

  void StartDebug(mjlib::io::ErrorCallback callback) {
    factory_->AsyncCreate(
        debug_stream_,
        std::bind(&Impl::HandleDebugStream, this, pl::_1, pl::_2));

    boost::asio::post(executor_, std::bind(std::move(callback),
                                           mjlib::base::error_code()));
  }

  void HandleDebugStream(const mjlib::base::error_code& ec,
                         mjlib::io::SharedStream stream) {
    mjlib::base::FailIf(ec);

    quad_debug_ = std::make_unique<QuadrupedDebug>(
        m_.quadruped_control.get(), stream);
  }

  boost::asio::executor executor_;
  mjlib::io::StreamFactory* const factory_;

  base::LogRef log_ = base::GetLogInstance("Quadruped");

  Members m_;
  Parameters p_;

  mjlib::io::StreamFactory::Options debug_stream_;

  std::unique_ptr<QuadrupedDebug> quad_debug_;
};

Quadruped::Quadruped(base::Context& context)
    : impl_(std::make_unique<Impl>(context)) {}

Quadruped::~Quadruped() {}

void Quadruped::AsyncStart(mjlib::io::ErrorCallback callback) {
  impl_->AsyncStart(std::move(callback));
}

clipp::group Quadruped::program_options() {
  return (
      mjlib::base::ClippArchive().Accept(&impl_->p_).release(),
      mjlib::base::ClippArchive("debug.")
      .Accept(&impl_->debug_stream_).release(),
      base::ClippComponentArchive().Accept(&impl_->m_).release()
  );
}

}
}

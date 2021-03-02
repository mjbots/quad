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

#include <boost/asio/post.hpp>

#include "base/logging.h"
#include "base/telemetry_registry.h"
#include "mech/pi3hat_wrapper.h"

namespace pl = std::placeholders;

namespace mjmech {
namespace mech {

class Quadruped::Impl {
 public:
  Impl(base::Context& context)
      : executor_(context.executor),
        factory_(context.factory.get()),
        telemetry_registry_(context.telemetry_registry.get()) {
    m_.pi3hat = std::make_unique<
      mjlib::io::Selector<Pi3hatInterface>>(executor_, "type");
    m_.pi3hat->Register<Pi3hatWrapper>("pi3hat");
    m_.pi3hat->set_default("pi3hat");

    m_.quadruped_control = std::make_unique<QuadrupedControl>(
        context,
        [&]() { return m_.pi3hat->selected(); } );
    m_.web_control = std::make_unique<QuadrupedWebControl>(
        context.executor,
        [q=m_.quadruped_control.get()](const auto& cmd) {
          q->Command(cmd);
        },
        [q=m_.quadruped_control.get()]() {
          return q->status();
        },
        []() {
          QuadrupedWebControl::Options options;
          options.asset_path = "web_control_assets";
          return options;
        }());
    m_.rf_control = std::make_unique<RfControl>(
        context, m_.quadruped_control.get(),
        [&]() { return m_.pi3hat->selected(); } );
    m_.system_info = std::make_unique<SystemInfo>(context);
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    base::StartArchive::Start(
        &m_, [this, callback=std::move(callback)](
            const mjlib::base::error_code& ec) mutable {
               // pi3hat should be initialized by now
               Pi3hatWrapper* const pi3hat =
                   dynamic_cast<Pi3hatWrapper*>(m_.pi3hat->selected());
               if (pi3hat) {
                 log_.warn("Registering power");
                 telemetry_registry_->Register("power", pi3hat->power_signal());
               }
               std::move(callback)(ec);
             });
  }

  boost::asio::any_io_executor executor_;
  mjlib::io::StreamFactory* const factory_;
  base::TelemetryRegistry* telemetry_registry_;

  base::LogRef log_ = base::GetLogInstance("Quadruped");

  Members m_;
  Parameters p_;
};

Quadruped::Quadruped(base::Context& context)
    : impl_(std::make_unique<Impl>(context)) {}

Quadruped::~Quadruped() {}

void Quadruped::AsyncStart(mjlib::io::ErrorCallback callback) {
  impl_->AsyncStart(std::move(callback));
}

Quadruped::Members* Quadruped::m() { return &impl_->m_; }

clipp::group Quadruped::program_options() {
  return (
      mjlib::base::ClippArchive().Accept(&impl_->p_).release(),
      base::ClippComponentArchive().Accept(&impl_->m_).release()
  );
}

}
}

// Copyright 2020 Josh Pieper, jjp@pobox.com.
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

#include "mech/turret.h"

#include <boost/asio/post.hpp>

#include "base/logging.h"

#include "mech/pi3hat_wrapper.h"

namespace pl = std::placeholders;

namespace mjmech {
namespace mech {

class Turret::Impl {
 public:
  Impl(base::Context& context)
      : executor_(context.executor),
        factory_(context.factory.get()) {
    {
      Pi3hatWrapper::Options hat_options;
      hat_options.mounting.yaw_deg = 0;
      hat_options.mounting.pitch_deg = 90;
      hat_options.mounting.roll_deg = 90;
      hat_options.rf_id = 88754;

      m_.pi3hat = std::make_unique<
        mjlib::io::Selector<Pi3hatInterface>>(executor_, "type");
      m_.pi3hat->Register<Pi3hatWrapper>("pi3", hat_options);
      m_.pi3hat->set_default("pi3");
    }

    m_.turret_control = std::make_unique<TurretControl>(
        context,
        [&]() { return m_.pi3hat->selected(); },
        [&]() { return m_.pi3hat->selected(); } );
    m_.web_control = std::make_unique<TurretWebControl>(
        context.executor,
        [t=m_.turret_control.get()](const auto& cmd) {
          t->Command(cmd);
        },
        [t=m_.turret_control.get()]() {
          return t->status();
        },
        []() {
          TurretWebControl::Options options;
          options.asset_path = "turret_assets";
          return options;
        }());
    m_.rf_control = std::make_unique<TurretRfControl>(
        context, m_.turret_control.get(),
        [&]() { return m_.pi3hat->selected(); });
    m_.system_info = std::make_unique<SystemInfo>(context);
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    base::StartArchive::Start(&m_, std::move(callback));
  }

  boost::asio::executor executor_;
  mjlib::io::StreamFactory* const factory_;

  base::LogRef log_ = base::GetLogInstance("Turret");

  Members m_;
  Parameters p_;
};

Turret::Turret(base::Context& context)
    : impl_(std::make_unique<Impl>(context)) {}

Turret::~Turret() {}

void Turret::AsyncStart(mjlib::io::ErrorCallback callback) {
  impl_->AsyncStart(std::move(callback));
}

Turret::Members* Turret::m() { return &impl_->m_; }

clipp::group Turret::program_options() {
  return (
      mjlib::base::ClippArchive().Accept(&impl_->p_).release(),
      base::ClippComponentArchive().Accept(&impl_->m_).release()
  );
}

}
}

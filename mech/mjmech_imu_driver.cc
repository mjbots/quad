// Copyright 2014-2019 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#define BOOST_BIND_NO_PLACEHOLDERS

#include "mjmech_imu_driver.h"

#include <fmt/format.h>

#include "mjlib/base/fail.h"
#include "mjlib/base/system_error.h"

#include "base/common.h"
#include "base/i2c_factory.h"
#include "base/now.h"
#include "base/quaternion.h"

using namespace std::placeholders;

namespace mjmech {
namespace mech {

namespace {
auto to_int16 = [](uint8_t msb, uint8_t lsb) {
  int32_t result = msb * 256 + lsb;
  if (result > 32767) { result = result - 65536; }
  return static_cast<int16_t>(result);
};

const double kGravity = 9.80665;

struct Parameters {
  std::unique_ptr<base::I2CFactory::Parameters> i2c;
  int gyro_address = 0x59;
  int accel_address = 0x1d;

  double accel_g = 4.0;
  double rotation_dps = 500.0;
  double rate_hz = 100.0;

  double roll_deg = 0;
  double pitch_deg = 0;
  double yaw_deg = 0;
  base::Point3D gyro_scale = base::Point3D(1, 1, 1);
  base::Point3D accel_scale = base::Point3D(1, 1, 1);

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(i2c));
    a->Visit(MJ_NVP(gyro_address));
    a->Visit(MJ_NVP(accel_address));
    a->Visit(MJ_NVP(accel_g));
    a->Visit(MJ_NVP(rotation_dps));
    a->Visit(MJ_NVP(rate_hz));
    a->Visit(MJ_NVP(roll_deg));
    a->Visit(MJ_NVP(pitch_deg));
    a->Visit(MJ_NVP(yaw_deg));
    a->Visit(MJ_NVP(gyro_scale));
    a->Visit(MJ_NVP(accel_scale));
  }
};
}

class MjmechImuDriver::Impl : boost::noncopyable {
 public:
  Impl(MjmechImuDriver* parent, boost::asio::io_service& service,
       base::I2CFactory* i2c_factory)
      : parent_(parent),
        service_(service),
        i2c_factory_(i2c_factory) {
    parameters_.i2c = i2c_factory->MakeParameters();
  }

  ~Impl() {
  }

  void AsyncStart(mjlib::io::ErrorCallback handler) {
    transform_ = base::Quaternion::FromEuler(
        base::Radians(parameters_.roll_deg),
        base::Radians(parameters_.pitch_deg),
        base::Radians(parameters_.yaw_deg));

    i2c_factory_->AsyncCreate(
        *parameters_.i2c,
        std::bind(&Impl::HandleStart, this, handler, _1, _2));
  }

  boost::program_options::options_description* options() { return &options_; }

 private:
  void HandleStart(mjlib::io::ErrorCallback handler,
                   mjlib::base::error_code ec,
                   base::SharedI2C i2c) {
    if (ec) {
      ec.Append("when starting imu i2c");
      service_.post(std::bind(handler, ec));
      return;
    }

    i2c_ = i2c;

    InitializeImu(handler);
  }

  void InitializeImu(mjlib::io::ErrorCallback handler) {
    ReadGyro(0x20, 1, std::bind(&Impl::HandleWhoAmI, this, handler, _1, _2));
  }

  void HandleWhoAmI(mjlib::io::ErrorCallback handler,
                    mjlib::base::error_code ec, std::size_t) {
    if (ec) {
      ec.Append("when requesting gyro whoami");
      service_.post(std::bind(handler, ec));
      return;
    }

    const uint8_t whoami = buffer_[0];
    if (whoami != 0xb1) {
      auto error = mjlib::base::error_code::einval(
          fmt::format("Incorrect whoami got 0x{:02X} expected 0xb1",
                      static_cast<int>(whoami)));
      service_.post(std::bind(handler, error));
      return;
    }

    ReadAccel(0x20, 1,
              std::bind(&Impl::HandleInitAccelRead, this, handler, _1, _2));
  }

  void HandleInitAccelRead(mjlib::io::ErrorCallback handler,
                           mjlib::base::error_code ec, std::size_t) {
    if (ec) {
      ec.Append("when discovering accelerometer");
      service_.post(std::bind(handler, ec));
      return;
    }

    try {
      // Configure the gyroscope.
      const uint8_t fs = [&]() {
        const double scale = parameters_.rotation_dps;
        if (scale <= 250) { return 3; }
        else if (scale <= 500) { return 2; }
        else if (scale <= 1000) { return 1; }
        else if (scale <= 2000) { return 0; }
        throw mjlib::base::system_error::einval("scale too large");
      }();

      imu_config_.rotation_dps = [fs]() {
        switch (fs) {
          case 0: { return 2000.0; }
          case 1: { return 1000.0; }
          case 2: { return 500.0; }
          case 3: { return 250.0; }
        }
        throw mjlib::base::system_error::einval("invalid state");
      }();

      // FS=XX PWR=Normal XYZ=EN
      buffer_[0] = static_cast<uint8_t>(0x0f | (fs << 6));

      // Pick a bandwidth which is less than half the desired rate.
      const uint8_t bw = [&]() {
        const double rate = parameters_.rate_hz;
        if (rate < 8) { return 0; } // 2 Hz
        else if (rate < 12) { return 1; } // 4 Hz
        else if (rate < 16) { return 2; } // 6 Hz
        else if (rate < 20) { return 3; } // 8 Hz
        else if (rate < 28) { return 4; } // 10 Hz
        else if (rate < 44) { return 5; } // 14 Hz
        else if (rate < 64) { return 6; } // 22 Hz
        else if (rate < 100) { return 7; } // 32 Hz
        else if (rate < 150) { return 8; } // 50 Hz
        else if (rate < 200) { return 9; } // 75 Hz
        else if (rate < 300) { return 10; } // 100 Hz
        else if (rate < 400) { return 11; } // 150 Hz
        else if (rate < 500) { return 12; } // 200 Hz
        else if (rate < 600) { return 13; } // 250 Hz
        else if (rate < 800) { return 14; } // 300 Hz
        else { return 15; } // 400 Hz
      }();

      imu_config_.gyro_bw_hz = [bw]() {
        switch (bw) {
          case 0: { return 2.0; }
          case 1: { return 4.0; }
          case 2: { return 6.0; }
          case 3: { return 8.0; }
          case 4: { return 10.0; }
          case 5: { return 14.0; }
          case 6: { return 22.0; }
          case 7: { return 32.0; }
          case 8: { return 50.0; }
          case 9: { return 75.0; }
          case 10: { return 100.0; }
          case 11: { return 150.0; }
          case 12: { return 200.0; }
          case 13: { return 250.0; }
          case 14: { return 300.0; }
          case 15: { return 400.0; }
        }
        throw mjlib::base::system_error::einval("invalid state");
      }();

      buffer_[1] = static_cast<uint8_t>(0x00 | (bw << 2)); // BW=X

      auto limit = [](double value) -> uint8_t {
        if (value < 0) { return 0; }
        if (value > 255) { return 255; }
        return static_cast<uint8_t>(value);
      };

      const uint8_t odr = limit([&]() {
          const double rate = parameters_.rate_hz;
          if (rate < 4.95) {
            throw mjlib::base::system_error::einval("IMU rate too small");
          } else if (rate < 20) {
            return (154 * rate + 500) / rate;
          } else if (rate < 100) {
            return (79 * rate + 2000) / rate;
          } else if (rate < 10000) {
            return (10000 - rate) / rate;
          } else {
            throw mjlib::base::system_error::einval("IMU rate too large");
          }
        }());

      buffer_[2] = odr; // ODR=100Hz

      imu_config_.rate_hz = [odr]() {
        if (odr <= 99) { return 10000.0 / (odr + 1); }
        else if (odr <= 179) { return 10000.0 / (100 + 5 * (odr - 99)); }
        else if (odr <= 255) { return 10000.0 / (500 + 20 * (odr - 179)); }
        throw mjlib::base::system_error::einval("invalid state");
      }();
    } catch (mjlib::base::system_error& se) {
      ec.Append("when configuring gyro");
      service_.post(std::bind(handler, se.code()));
      return;
    }

    WriteGyro(0x00, 3,
              std::bind(&Impl::HandleGyroConfig, this, handler, _1, _2));
  }

  void HandleGyroConfig(mjlib::io::ErrorCallback handler,
                        mjlib::base::error_code ec, std::size_t) {
    if (ec) {
      ec.Append("when configuring gyro");
      service_.post(std::bind(handler, ec));
      return;
    }

    // Configure the accelerometer.
    // Turn it off so we can change settings.
    buffer_[0] = 0x18;
    WriteAccel(0x2a, 1,
               std::bind(&Impl::HandleAccelDisable, this, handler, _1, _2));
  }

  void HandleAccelDisable(mjlib::io::ErrorCallback handler,
                          mjlib::base::error_code ec, std::size_t) {
    if (ec) {
      ec.Append("when disabling accelerometer");
      service_.post(std::bind(handler, ec));
      return;
    }

    try {

      const uint8_t fsg = [&]() {
        const double scale = parameters_.accel_g;
        if (scale <= 2.0) { return 0; }
        else if (scale <= 4.0) { return 1; }
        else if (scale <= 8.0) { return 2; }
        throw mjlib::base::system_error::einval("accel scale too large");
      }();

      imu_config_.accel_g = [fsg]() {
        switch (fsg) {
          case 0: { return 2.0; }
          case 1: { return 4.0; }
          case 2: { return 8.0; }
        }
        throw mjlib::base::system_error::einval("invalid state");
      }();

      buffer_[0] = fsg;
    } catch (mjlib::base::system_error& se) {
      service_.post(std::bind(handler, se.code()));
      return;
    }

    WriteAccel(0x0e, 1,
               std::bind(&Impl::HandleAccelFS, this, handler, _1, _2));
  }

  void HandleAccelFS(mjlib::io::ErrorCallback handler,
                     mjlib::base::error_code ec, std::size_t) {
    if (ec) {
      ec.Append("when setting acclerometer scale");
      service_.post(std::bind(handler, ec));
      return;
    }

    try {
      const uint8_t dr = [this]() {
        const double rate = parameters_.rate_hz;
        if (rate <= 1.56) { return 7; }
        else if (rate <= 6.25) { return 6; }
        else if (rate <= 12.5) { return 5; }
        else if (rate <= 50) { return 4; }
        else if (rate <= 100) { return 3; }
        else if (rate <= 200) { return 2; }
        else if (rate <= 400) { return 1; }
        else if (rate <= 800) { return 0; }
        throw mjlib::base::system_error::einval("accel rate too large");
      }();
      const uint8_t lnoise = [&]() {
        if (parameters_.accel_g <= 4) { return 1; }
        return 0;
      }();

      // 2A 0b00011000 ASLP=0 ODR=XXHz LNOISE=0 F_READ=0 ACTIVE=0
      // 2B 0b00000010 ST=0 RST=0 SMODS=0 SLPE=0 MODS=2
      buffer_[0] = static_cast<uint8_t>(0x00 | (dr << 3) | (lnoise << 2));
      buffer_[1] = 0x02;
    } catch (mjlib::base::system_error& se) {
      service_.post(std::bind(handler, se.code()));
      return;
    }
    WriteAccel(0x2a, 1,
               std::bind(&Impl::HandleAccelConfig, this, handler, _1, _2));
  }

  void HandleAccelConfig(mjlib::io::ErrorCallback handler,
                         mjlib::base::error_code ec, std::size_t) {
    if (ec) {
      ec.Append("when configuring accelerometer");
      service_.post(std::bind(handler, ec));
      return;
    }

    // Turn it back on.
    buffer_[0] = 0x19;
    WriteAccel(0x2a, 1,
               std::bind(&Impl::HandleAccelEnable, this, handler, _1, _2));
  }

  void HandleAccelEnable(mjlib::io::ErrorCallback handler,
                         mjlib::base::error_code ec, std::size_t) {
    if (ec) {
      ec.Append("when enabling accelerometer");
      service_.post(std::bind(handler, ec));
      return;
    }

    imu_config_.roll_deg = parameters_.roll_deg;
    imu_config_.pitch_deg = parameters_.pitch_deg;
    imu_config_.yaw_deg = parameters_.yaw_deg;
    imu_config_.gyro_scale = parameters_.gyro_scale;
    imu_config_.accel_scale = parameters_.accel_scale;

    imu_config_.timestamp = base::Now(service_);

    // Emit our config.
    parent_->imu_config_signal_(&imu_config_);

    service_.post(std::bind(handler, mjlib::base::error_code()));

    accel_sensitivity_ = [&]() {
      const double fs = imu_config_.accel_g;
      if (fs == 2.0) { return 1.0 / 4096 / 4; }
      else if (fs == 4.0) { return 1.0 / 2048 / 4; }
      else if (fs == 8.0) { return 1.0 / 1024 / 4; }
      mjlib::base::Fail("invalid configured accel range");
    }();

    gyro_sensitivity_ = [&]() {
      const double fs = imu_config_.rotation_dps;
      if (fs == 250.0) { return 1.0 / 120.0; }
      else if (fs == 500.0) { return 1.0 / 60.0; }
      else if (fs == 1000.0) { return 1.0 / 30.0; }
      else if (fs == 2000.0) { return 1.0 / 15.0; }
      mjlib::base::Fail("invalid configured gyro rate");
    }();

    StartDataLoop();
  }

  void StartDataLoop() {
    ReadGyro(0x22, 1, std::bind(&Impl::HandleGyroPoll, this, _1, _2));
  }

  void HandleGyroPoll(mjlib::base::error_code ec, std::size_t) {
    mjlib::base::FailIf(ec);
    if ((buffer_[0] & 0x01) == 0) {
      StartDataLoop();
    } else {
      StartDataRead();
    }
  }

  void StartDataRead() {
    ReadGyro(0x22, 7, std::bind(&Impl::HandleGyroRead, this, _1, _2));
  }

  void HandleGyroRead(mjlib::base::error_code ec, std::size_t) {
    mjlib::base::FailIf(ec);


    const auto& gdata = buffer_;

    base::Point3D body_rate_dps;
    body_rate_dps.x() =
        to_int16(gdata[1], gdata[2]) * gyro_sensitivity_ *
        parameters_.gyro_scale.x();
    body_rate_dps.y() =
        to_int16(gdata[3], gdata[4]) * gyro_sensitivity_ *
        parameters_.gyro_scale.y();
    body_rate_dps.z() =
        to_int16(gdata[5], gdata[6]) * gyro_sensitivity_ *
        parameters_.gyro_scale.z();

    auto rate_dps = transform_.Rotate(body_rate_dps);

    ReadAccel(0x00, 7,
              std::bind(&Impl::HandleAccelRead, this, rate_dps, _1, _2));
  }

  void HandleAccelRead(base::Point3D rate_dps,
                       mjlib::base::error_code ec, std::size_t) {
    mjlib::base::FailIf(ec);

    ImuData imu_data;
    imu_data.timestamp = base::Now(service_);

    const auto& adata = buffer_;
    base::Point3D accel_mps2;

    accel_mps2.x() =
        to_int16(adata[1], adata[2]) * accel_sensitivity_ * kGravity *
        parameters_.accel_scale.x();
    accel_mps2.y() =
          to_int16(adata[3], adata[4]) * accel_sensitivity_ * kGravity *
        parameters_.accel_scale.y();
    accel_mps2.z() =
          to_int16(adata[5], adata[6]) * accel_sensitivity_ * kGravity *
        parameters_.accel_scale.z();

    imu_data.accel_mps2 = transform_.Rotate(accel_mps2);

    imu_data.body_rate_dps = rate_dps;

    parent_->imu_data_signal_(&imu_data);

    StartDataLoop();
  }

  void WriteGyro(uint8_t reg, std::size_t length,
                 mjlib::io::WriteHandler handler) {
    WriteData(parameters_.gyro_address, reg, length, handler);
  }

  void WriteAccel(uint8_t reg, std::size_t length,
                  mjlib::io::WriteHandler handler) {
    WriteData(parameters_.accel_address, reg, length, handler);
  }

  void ReadGyro(uint8_t reg, std::size_t length,
                mjlib::io::ReadHandler handler) {
    ReadData(parameters_.gyro_address, reg, length, handler);
  }

  void ReadAccel(uint8_t reg, std::size_t length,
                 mjlib::io::ReadHandler handler) {
    ReadData(parameters_.accel_address, reg, length, handler);
  }

  void WriteData(uint8_t address, uint8_t reg, std::size_t length,
                 mjlib::io::WriteHandler handler) {

    i2c_->AsyncWrite(address, reg, boost::asio::buffer(buffer_, length),
                     handler);
  }

  void ReadData(uint8_t address, uint8_t reg, std::size_t length,
                mjlib::io::ReadHandler handler) {

    i2c_->AsyncRead(address, reg, boost::asio::buffer(buffer_, length),
                    handler);
  }

  // From both.
  MjmechImuDriver* const parent_;
  Parameters parameters_;
  boost::program_options::options_description options_;
  boost::asio::io_service& service_;
  base::I2CFactory* const i2c_factory_;
  base::SharedI2C i2c_;

  uint8_t buffer_[256] = {};

  ImuConfig imu_config_;
  base::Quaternion transform_;

  double accel_sensitivity_ = 0.0;
  double gyro_sensitivity_ = 0.0;
};

MjmechImuDriver::MjmechImuDriver(boost::asio::io_service& service,
                                 base::I2CFactory* i2c_factory)
    : impl_(new Impl(this, service, i2c_factory)) {
}

MjmechImuDriver::~MjmechImuDriver() {}

void MjmechImuDriver::AsyncStart(mjlib::io::ErrorCallback handler) {
  impl_->AsyncStart(handler);
}

boost::program_options::options_description*
MjmechImuDriver::options() {
  return impl_->options();
}

}
}

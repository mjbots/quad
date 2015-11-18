// Copyright 2014-2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "mjmech_imu_driver.h"

#include <linux/i2c-dev.h>

#include <thread>

#include "base/common.h"
#include "base/fail.h"
#include "base/quaternion.h"

namespace mjmech {
namespace mech {

namespace {
int ErrWrap(int value) {
  if (value < 0) {
    throw base::SystemError(errno, boost::system::generic_category());
  }
  return value;
}
}

class MjmechImuDriver::Impl : boost::noncopyable {
 public:
  Impl(MjmechImuDriver* parent, boost::asio::io_service& service)
      : parent_(parent),
        service_(service),
        work_(service_),
        parent_id_(std::this_thread::get_id()) {}

  ~Impl() {
    done_ = true;
    if (child_.joinable()) { child_.join(); }
  }

  void AsyncStart(base::ErrorHandler handler) {
    BOOST_ASSERT(std::this_thread::get_id() == parent_id_);

    // Capture our parent's parameters before starting our thread.
    parameters_ = parent_->parameters_;

    child_ = std::thread(std::bind(&Impl::Run, this, handler));
  }

 private:
  void Run(base::ErrorHandler handler) {
    BOOST_ASSERT(std::this_thread::get_id() == child_.get_id());

    transform_ = base::Quaternion::FromEuler(
        base::Radians(parameters_.roll_deg),
        base::Radians(parameters_.pitch_deg),
        base::Radians(parameters_.yaw_deg));

    try {
      // Open the I2C device and configure it.
      fd_ = ::open(parameters_.i2c_device.c_str(), O_RDWR);
      if (fd_ < 0) {
        throw base::SystemError(
            errno, boost::system::generic_category(),
            "error opening '" + parameters_.i2c_device + "'");
      }

      // Verify we can talk to the gyro.
      uint8_t whoami[1] = {};
      ReadGyro(0x20, 1, whoami, sizeof(whoami));
      if (whoami[0] != 0xb1) {
        throw base::SystemError::einval(
            (boost::format("Incorrect whoami got 0x%02X expected 0xb1") %
             static_cast<int>(whoami[0])).str());
      }

      // Verify we can talk to the accelerometer.
      uint8_t ignored[1] = {};
      ReadAccel(0x20, 1, ignored, sizeof(ignored));

      imu_config_.timestamp =
          boost::posix_time::microsec_clock::universal_time();

      // Configure the gyroscope.
      const uint8_t fs = [this]() {
        const double scale = parameters_.rotation_deg_s;
        if (scale <= 250) { return 3; }
        else if (scale <= 500) { return 2; }
        else if (scale <= 1000) { return 1; }
        else if (scale <= 2000) { return 0; }
        throw base::SystemError::einval("scale too large");
      }();

      imu_config_.rotation_deg_s = [fs]() {
        switch (fs) {
          case 0: { return 2000.0; }
          case 1: { return 1000.0; }
          case 2: { return 500.0; }
          case 3: { return 250.0; }
        }
        throw base::SystemError::einval("invalid state");
      }();

      // FS=XX PWR=Normal XYZ=EN
      WriteGyro(0x00, {static_cast<uint8_t>(0x0f | (fs << 6))});

      // Pick a bandwidth which is less than half the desired rate.
      const uint8_t bw = [this]() {
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
        throw base::SystemError::einval("invalid state");
      }();

      WriteGyro(0x01, {static_cast<uint8_t>(0x00 | (bw << 2))}); // BW=X

      auto limit = [](double value) -> uint8_t {
        if (value < 0) { return 0; }
        if (value > 255) { return 255; }
        return static_cast<uint8_t>(value);
      };

      const uint8_t odr = limit([this]() {
          const double rate = parameters_.rate_hz;
          if (rate < 4.95) {
            throw base::SystemError::einval("IMU rate too small");
          } else if (rate < 20) {
            return (154 * rate + 500) / rate;
          } else if (rate < 100) {
            return (79 * rate + 2000) / rate;
          } else if (rate < 10000) {
            return (10000 - rate) / rate;
          } else {
            throw base::SystemError::einval("IMU rate too large");
          }
        }());
      WriteGyro(0x02, {odr}); // ODR=100Hz

      imu_config_.rate_hz = [odr]() {
        if (odr <= 99) { return 10000.0 / (odr + 1); }
        else if (odr <= 179) { return 10000.0 / (100 + 5 * (odr - 99)); }
        else if (odr <= 255) { return 10000.0 / (500 + 20 * (odr - 179)); }
        throw base::SystemError::einval("invalid state");
      }();

      // Configure the accelerometer.
      WriteAccel(0x2a, {0x18}); // Turn it off so we can change settings.

      const uint8_t fsg = [this]() {
        const double scale = parameters_.accel_g;
        if (scale <= 2.0) { return 0; }
        else if (scale <= 4.0) { return 1; }
        else if (scale <= 8.0) { return 2; }
        throw base::SystemError::einval("accel scale too large");
      }();

      imu_config_.accel_g = [fsg]() {
        switch (fsg) {
          case 0: { return 2.0; }
          case 1: { return 4.0; }
          case 2: { return 8.0; }
        }
        throw base::SystemError::einval("invalid state");
      }();

      WriteAccel(0x0e, {fsg}); // FS=2G

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
        throw base::SystemError::einval("accel rate too large");
      }();
      const uint8_t lnoise = [this]() {
        if (parameters_.accel_g <= 4) { return 1; }
        return 0;
      }();

      // 2A 0b00011000 ASLP=0 ODR=XXHz LNOISE=0 F_READ=0 ACTIVE=0
      // 2B 0b00000010 ST=0 RST=0 SMODS=0 SLPE=0 MODS=2
      WriteAccel(0x2a, {
          static_cast<uint8_t>(0x00 | (dr << 3) | (lnoise << 2)),
              0x02});

      // Turn it back on.
      WriteAccel(0x2a, {0x19});

      imu_config_.roll_deg = parameters_.roll_deg;
      imu_config_.pitch_deg = parameters_.pitch_deg;
      imu_config_.yaw_deg = parameters_.yaw_deg;
      imu_config_.gyro_scale = parameters_.gyro_scale;
      imu_config_.accel_scale = parameters_.accel_scale;

      service_.post(std::bind(&Impl::SendConfig, this, imu_config_));

    } catch (base::SystemError& e) {
      e.error_code().Append("when initializing IMU");
      service_.post(std::bind(handler, e.error_code()));
      return;
    }

    service_.post(std::bind(handler, base::ErrorCode()));

    try {
      DataLoop();
    } catch (base::SystemError& e) {
      // TODO jpieper: We should somehow log this or do something
      // useful.  For now, just re-raise.
      throw;
    }
  }

  void DataLoop() {
    BOOST_ASSERT(std::this_thread::get_id() == child_.get_id());

    auto to_int16 = [](uint8_t msb, uint8_t lsb) {
      int32_t result = msb * 256 + lsb;
      if (result > 32767) { result = result - 65536; }
      return static_cast<int16_t>(result);
    };

    const double kAccelSensitivity = [this]() {
      const double fs = imu_config_.accel_g;
      if (fs == 2.0) { return 1.0 / 4096 / 4; }
      else if (fs == 4.0) { return 1.0 / 2048 / 4; }
      else if (fs == 8.0) { return 1.0 / 1024 / 4; }
      throw base::SystemError::einval("invalid configured accel range");
    }();

    const double kGyroSensitivity = [this]() {
      const double fs = imu_config_.rotation_deg_s;
      if (fs == 250.0) { return 1.0 / 120.0; }
      else if (fs == 500.0) { return 1.0 / 60.0; }
      else if (fs == 1000.0) { return 1.0 / 30.0; }
      else if (fs == 2000.0) { return 1.0 / 15.0; }
      throw base::SystemError::einval("invalid configured gyro rate");
    }();

    // Loop reading things and emitting data.
    while (!done_) {
      // Read gyro values until we get a new one.
      uint8_t data[10] = {};
      for (int i = 0; i < 15; i++) {
        ReadGyro(0x22, 1, data, sizeof(data));
        if ((data[0] & 0x01) != 0) { break; }
      }

      ImuData imu_data;
      imu_data.timestamp =
          boost::posix_time::microsec_clock::universal_time();

      uint8_t gdata[10] = {};
      ReadGyro(0x22, 7, gdata, sizeof(gdata));

      uint8_t adata[10] = {};
      ReadAccel(0x00, 7, adata, sizeof(adata));

      const double kGravity = 9.80665;

      base::Point3D accel_mps2;

      accel_mps2.x =
          to_int16(adata[1], adata[2]) * kAccelSensitivity * kGravity *
          parameters_.accel_scale.x;
      accel_mps2.y =
          to_int16(adata[3], adata[4]) * kAccelSensitivity * kGravity *
          parameters_.accel_scale.y;
      accel_mps2.z =
          to_int16(adata[5], adata[6]) * kAccelSensitivity * kGravity *
          parameters_.accel_scale.z;

      imu_data.accel_mps2 = transform_.Rotate(accel_mps2);

      base::Point3D body_rate_deg_s;
      body_rate_deg_s.x =
          to_int16(gdata[1], gdata[2]) * kGyroSensitivity *
          parameters_.gyro_scale.x;
      body_rate_deg_s.y =
          to_int16(gdata[3], gdata[4]) * kGyroSensitivity *
          parameters_.gyro_scale.y;
      body_rate_deg_s.z =
          to_int16(gdata[5], gdata[6]) * kGyroSensitivity *
          parameters_.gyro_scale.z;

      imu_data.body_rate_deg_s = transform_.Rotate(body_rate_deg_s);

      service_.post(std::bind(&Impl::SendData, this, imu_data));
    }
  }

  void SendData(const ImuData& imu_data) {
    BOOST_ASSERT(std::this_thread::get_id() == parent_id_);

    parent_->imu_data_signal_(&imu_data);
  }

  void SendConfig(const ImuConfig& imu_config) {
    BOOST_ASSERT(std::this_thread::get_id() == parent_id_);

    parent_->imu_config_signal_(&imu_config);
  }

  void WriteGyro(uint8_t reg, const std::vector<uint8_t>& data) {
    WriteData(parameters_.gyro_address, reg, data);
  }

  void WriteAccel(uint8_t reg, const std::vector<uint8_t>& data) {
    WriteData(parameters_.accel_address, reg, data);
  }

  void ReadGyro(uint8_t reg, std::size_t length,
                uint8_t* buffer, std::size_t buffer_size) {
    ReadData(parameters_.gyro_address, reg, length,
             buffer, buffer_size);
  }

  void ReadAccel(uint8_t reg, std::size_t length,
                uint8_t* buffer, std::size_t buffer_size) {
    ReadData(parameters_.accel_address, reg, length,
             buffer, buffer_size);
  }

  void WriteData(uint8_t address, uint8_t reg,
                 const std::vector<uint8_t>& data) {
    BOOST_ASSERT(std::this_thread::get_id() == child_.get_id());

    try {
      ErrWrap(::ioctl(fd_, I2C_SLAVE, static_cast<int>(address)));

      union i2c_smbus_data i2c_data;
      BOOST_ASSERT(data.size() <= 32);
      i2c_data.block[0] = data.size();
      for (size_t i = 0; i < data.size(); ++i) {
        i2c_data.block[i + 1] = data[i];
      }
      ErrWrap(::i2c_smbus_access(fd_, I2C_SMBUS_WRITE, reg,
                                 I2C_SMBUS_I2C_BLOCK_BROKEN, &i2c_data));
    } catch (base::SystemError& se) {
      se.error_code().Append(boost::format("WriteData(0x%02x, 0x%02x, ...)") %
                             static_cast<int>(address) %
                             static_cast<int>(reg));
      throw;
    }
  }

  void ReadData(uint8_t address, uint8_t reg, std::size_t length,
                uint8_t* buffer, std::size_t buffer_length) {
    BOOST_ASSERT(std::this_thread::get_id() == child_.get_id());
    BOOST_ASSERT(buffer_length >= length);

    try {
      try {
        ErrWrap(::ioctl(fd_, I2C_SLAVE, static_cast<int>(address)));
      } catch (base::SystemError& se) {
        se.error_code().Append("when setting address");
        throw;
      }

      union i2c_smbus_data data;
      BOOST_ASSERT(length <= 255);
      data.block[0] = length;
      try {
        ErrWrap(::i2c_smbus_access(fd_, I2C_SMBUS_READ, reg,
                                   I2C_SMBUS_I2C_BLOCK_DATA, &data));
      } catch (base::SystemError& se) {
        se.error_code().Append("during transfer");
        throw;
      }

      if (data.block[0] != length) {
        throw base::SystemError(
            EINVAL, boost::system::generic_category(),
            (boost::format("asked for length %d got %d") %
             length % static_cast<int>(data.block[0])).str());
      }
      std::memcpy(buffer, &data.block[1], length);
    } catch (base::SystemError& se) {
      se.error_code().Append(boost::format("ReadData(0x%02x, 0x%02x, %d)") %
                             static_cast<int>(address) %
                             static_cast<int>(reg) %
                             length);
      throw;
    }
  }


  // From both.
  MjmechImuDriver* const parent_;
  boost::asio::io_service& service_;
  boost::asio::io_service::work work_;
  Parameters parameters_;

  const std::thread::id parent_id_;
  std::thread child_;
  bool done_ = false;

  // From child only.
  int fd_ = -1;
  ImuConfig imu_config_;
  base::Quaternion transform_;
};

MjmechImuDriver::MjmechImuDriver(boost::asio::io_service& service)
    : impl_(new Impl(this, service)) {}
MjmechImuDriver::~MjmechImuDriver() {}

void MjmechImuDriver::AsyncStart(base::ErrorHandler handler) {
  impl_->AsyncStart(handler);
}

}
}

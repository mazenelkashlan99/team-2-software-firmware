#pragma once
#include <stdint.h>
#include <Wire.h>
#include "ImuData.h"      // resolves to include/ImuData.h

// MPU6050 driver. I2C only. Single device, address 0x68.
//
// Gyro: degrees/sec internally, radians/sec out.
// Accel: g internally, m/s^2 out.
// Temperature: Celsius.
//
// The gyro's Z axis has a software bias removed from every sample.
// Calibrate on a still robot at boot; the bias is not persisted.
class Imu {
public:
    // Bring up I2C on the given pins. Returns false if the WHO_AM_I
    // check fails or any register write fails.
    bool begin(int sda, int scl);

    // Read one sample. Returns false on I2C error (leaves `out` alone).
    bool read(ImuData& out);

    // Average `durationMs` of samples with the robot still. Must be
    // called after begin() and before the first read() is trusted.
    // Blocks for the duration. Returns false if too few samples arrived.
    bool calibrateGyro(uint16_t durationMs);

    float gzBias() const { return gzBias_; }

private:
    bool writeReg(uint8_t reg, uint8_t val);
    bool readBytes(uint8_t reg, uint8_t* buf, uint8_t len);

    uint8_t whoAmI_ = 0;
    float   gzBias_ = 0.0f;
};
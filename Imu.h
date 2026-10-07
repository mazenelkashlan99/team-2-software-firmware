#pragma once
#include <Arduino.h>
#include <Wire.h>

struct ImuData {
  float ax, ay, az;   
  float gx, gy, gz;   
  float tempC;
  uint32_t tUs;
};

class Imu {
public:
  bool begin(int sda = 21, int scl = 22);
  bool read(ImuData &out);
  bool calibrateGyro(uint16_t durationMs = 1500);

  uint8_t lastWhoAmI() const { return whoAmI_; }
  float gyroBiasZ() const { return gzBias_; }

private:
  bool writeReg(uint8_t reg, uint8_t val);
  bool readBytes(uint8_t reg, uint8_t *buf, uint8_t len);

  uint8_t whoAmI_ = 0;
  float gzBias_ = 0.0f;
};
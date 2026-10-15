#include "Imu.h"
#include "imu_regs.h"

namespace {
constexpr float G_TO_MS2_FACTOR = 9.80665f;
constexpr float DEG_TO_RAD_FACTOR = 0.017453292519943f;
constexpr float GYRO_SCALE = DEG_TO_RAD_FACTOR / GYRO_LSB_PER_DPS;
constexpr float ACCEL_SCALE = G_TO_MS2_FACTOR / ACCEL_LSB_PER_G;

int16_t toInt16(uint8_t hi, uint8_t lo) {
    return (int16_t)(((uint16_t)hi << 8) | lo);
}
}

bool Imu::writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool Imu::readBytes(uint8_t reg, uint8_t* buf, uint8_t len) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)MPU_ADDR, (int)len) != len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}

bool Imu::begin(int sda, int scl) {
    Wire.begin(sda, scl);
    Wire.setClock(400000);

    if (!readBytes(REG_WHO_AM_I, &whoAmI_, 1) || whoAmI_ != 0x68) return false;

    if (!writeReg(REG_PWR_MGMT_1, PWR_CLK_GYRO_PLL)) return false;
    delay(10);
    if (!writeReg(REG_SMPLRT_DIV, SMPLRT_200HZ)) return false;
    if (!writeReg(REG_CONFIG, DLPF_42HZ)) return false;
    if (!writeReg(REG_GYRO_CONFIG, GYRO_FS_500)) return false;
    if (!writeReg(REG_ACCEL_CONFIG, ACCEL_FS_4G)) return false;

    return true;
}

bool Imu::read(ImuData& out) {
    uint8_t b[14];
    if (!readBytes(REG_ACCEL_XOUT_H, b, 14)) return false;

    out.ax = toInt16(b[0], b[1]) * ACCEL_SCALE;
    out.ay = toInt16(b[2], b[3]) * ACCEL_SCALE;
    out.az = toInt16(b[4], b[5]) * ACCEL_SCALE;
    out.tempC = toInt16(b[6], b[7]) / 340.0f + 36.53f;
    out.gx = toInt16(b[8], b[9]) * GYRO_SCALE;
    out.gy = toInt16(b[10], b[11]) * GYRO_SCALE;
    out.gz = toInt16(b[12], b[13]) * GYRO_SCALE - gzBias_;
    out.tUs = micros();
    return true;
}

bool Imu::calibrateGyro(uint16_t durationMs) {
    gzBias_ = 0.0f;
    double sum = 0.0;
    uint32_t n = 0;
    ImuData d;
    uint32_t start = millis();
    while (millis() - start < durationMs) {
        if (read(d)) { sum += d.gz; n++; }
        delay(5);
    }
    if (n < 50) return false;
    gzBias_ = (float)(sum / n);
    return true;
}
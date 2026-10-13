#pragma once
#include <stdint.h>

constexpr uint8_t MPU_ADDR = 0x68;  
constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
constexpr uint8_t REG_CONFIG       = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t REG_WHO_AM_I     = 0x75;
constexpr uint8_t PWR_CLK_GYRO_PLL = 0x01;
constexpr uint8_t DLPF_42HZ        = 0x03;
constexpr uint8_t GYRO_FS_500      = 0x08;  
constexpr uint8_t ACCEL_FS_4G      = 0x08;   
constexpr uint8_t SMPLRT_200HZ     = 4;
constexpr float GYRO_LSB_PER_DPS = 65.5f;
constexpr float ACCEL_LSB_PER_G  = 8192.0f;
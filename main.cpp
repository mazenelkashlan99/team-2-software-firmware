#include <Arduino.h>
#include "Imu.h"

Imu imu;

void setup() {
  Serial.begin(115200);
  delay(500);

  if (!imu.begin()) {
    Serial.printf("IMU NOT FOUND (WHO_AM_I = 0x%02X)\n", imu.lastWhoAmI());
    while (true) delay(1000);
  }
  Serial.println("IMU OK");

  Serial.println("Calibrating gyro, keep still...");
  if (imu.calibrateGyro()) Serial.println("Calibration done");
  else Serial.println("Calibration failed, try again");
}

void loop() {
  ImuData d;
  if (imu.read(d)) {
    Serial.printf("ax=%.2f ay=%.2f az=%.2f gz=%.3f temp=%.1f\n",
                  d.ax, d.ay, d.az, d.gz, d.tempC);
  }
  delay(100);
}
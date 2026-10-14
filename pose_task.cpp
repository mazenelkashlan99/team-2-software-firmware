#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "pose_task.h"
#include "Imu.h"
#include "Odometry.h"
#include "Fusion.h"

namespace {
constexpr uint32_t PERIOD_MS = 10;       
constexpr float GYRO_Z_SIGN = 1.0f;      

WheelController* wheelL = nullptr;
WheelController* wheelR = nullptr;
float trackWidth = 0.15f;

Imu imu;
Odometry odom;
Fusion fusion(0.98f);
SemaphoreHandle_t poseMutex = nullptr;
PoseData latest{};
bool havePose = false;

void poseTask(void*) {
  bool imuReady = imu.begin();
  if (!imuReady) {
    Serial.printf("[pose] IMU not found (WHO_AM_I = 0x%02X), using wheels only\n", imu.lastWhoAmI());
  } else if (!imu.calibrateGyro()) {
    Serial.println("[pose] gyro calibration failed (robot moved?), bias = 0");
  }

  float lastL = wheelL->getTotalDistance();
  float lastR = wheelR->getTotalDistance();
  uint32_t lastUs = micros();
  TickType_t wake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_MS));

    const uint32_t nowUs = micros();
    const float dt = (nowUs - lastUs) * 1e-6f;
    lastUs = nowUs;
    if (dt <= 0.0f) continue;

    const float totL = wheelL->getTotalDistance();
    const float totR = wheelR->getTotalDistance();
    const float dL = totL - lastL;
    const float dR = totR - lastR;
    lastL = totL;
    lastR = totR;

    ImuData d;
    const bool imuOk = imuReady && imu.read(d);
    const float gz = imuOk ? GYRO_Z_SIGN * d.gz : 0.0f;

    const WheelStep step = Odometry::wheelStep(dL, dR, trackWidth);

    xSemaphoreTake(poseMutex, portMAX_DELAY);
    const float dTheta = fusion.update(step.dTheta, gz, dt, imuOk);
    odom.integrate(step.ds, dTheta);
    latest.x = odom.x();
    latest.y = odom.y();
    latest.theta = odom.theta();
    latest.v = step.ds / dt;
    latest.omega = dTheta / dt;
    latest.tMs = millis();
    havePose = true;
    xSemaphoreGive(poseMutex);
  }
}
}  

void poseTaskStart(WheelController& left, WheelController& right, float trackWidthM) {
  wheelL = &left;
  wheelR = &right;
  trackWidth = trackWidthM;
  poseMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(poseTask, "pose", 4096, nullptr, 4, nullptr, 1);
}

bool getPose(PoseData& out) {
  if (!poseMutex) return false;
  xSemaphoreTake(poseMutex, portMAX_DELAY);
  out = latest;
  const bool ok = havePose;
  xSemaphoreGive(poseMutex);
  return ok;
}

void resetPose(float x, float y, float theta) {
  if (!poseMutex) return;
  xSemaphoreTake(poseMutex, portMAX_DELAY);
  odom.reset(x, y, theta);
  fusion.reset(theta);
  xSemaphoreGive(poseMutex);
}
#include "pose_task.h"
#include <Arduino.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "config.h"
#include "control_task.h"      // wheelDistanceLeft/Right()
#include "Imu.h"
#include "Odometry.h"
#include "Fusion.h"

namespace {

Imu      imu;
Odometry odom;
Fusion   fusion;

portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
robot::Pose  s_pose {};
bool s_imuReady = false;
uint32_t s_lastImuLogMs = 0;

}  // namespace

void poseInit() {
    Serial.printf("IMU init: MPU6050 SDA=GPIO%d SCL=GPIO%d address=0x68\n",
                  PIN_I2C_SDA, PIN_I2C_SCL);
    s_imuReady = imu.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    if (!s_imuReady) {
        Serial.println("IMU init failed; check sensor power, I2C wiring, and address.");
    } else if (!imu.calibrateGyro(1000)) {
        s_imuReady = false;
        Serial.println("IMU gyro calibration failed; keep the sensor still and check I2C.");
    } else {
        Serial.printf("IMU ready; gyro Z bias=%.4f rad/s\n", imu.gzBias());
    }
    odom.begin(TRACK_WIDTH_M);
    fusion.begin(0.98f);
}

robot::Pose poseGet() {
    robot::Pose p;
    portENTER_CRITICAL(&s_mux);
    p = s_pose;
    portEXIT_CRITICAL(&s_mux);
    return p;
}

void poseTask(void*) {
    esp_task_wdt_add(nullptr);
    TickType_t wake = xTaskGetTickCount();
    uint32_t   lastUs = micros();

    for (;;) {
        const uint32_t nowUs = micros();
        float dt = (nowUs - lastUs) * 1e-6f;
        lastUs = nowUs;
        if (dt < 0.005f) dt = 0.005f;
        if (dt > 0.050f) dt = 0.050f;

        // 1. IMU sample.
        ImuData imuSample{};
        const bool sampleOk = s_imuReady && imu.read(imuSample);
        const uint32_t nowMs = millis();
        if (nowMs - s_lastImuLogMs >= 1000) {
            s_lastImuLogMs = nowMs;
            if (sampleOk) {
                Serial.printf("IMU accel=(%+.2f,%+.2f,%+.2f) m/s^2 "
                              "gyro=(%+.3f,%+.3f,%+.3f) rad/s\n",
                              imuSample.ax, imuSample.ay, imuSample.az,
                              imuSample.gx, imuSample.gy, imuSample.gz);
            } else if (s_imuReady) {
                Serial.println("IMU read failed; check I2C wiring.");
            }
        }
        fusion.setDt(dt);

        // 2. Wheel deltas from control_task. These are written once per
        //    control tick and read here between ticks. Single floats are
        //    atomic on ESP32; no mutex needed.
        const float dL = wheelDistanceDeltaLeft();
        const float dR = wheelDistanceDeltaRight();

        // 3. Integrate wheel odometry.
        odom.update(dL, dR);

        // 4. Fuse with IMU. Returns a robot::Pose.
        robot::Pose p = fusion.update(odom.pose(), imuSample);
        p.t = millis();
        p.valid = sampleOk;

        portENTER_CRITICAL(&s_mux);
        s_pose = p;
        portEXIT_CRITICAL(&s_mux);

        esp_task_wdt_reset();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(POSE_PERIOD_MS));
    }
}

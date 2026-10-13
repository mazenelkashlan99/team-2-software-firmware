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

}  // namespace

void poseInit() {
    imu.begin();
    odom.begin();
    fusion.begin();
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
        const ImuSample imuSample = imu.read();

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
        p.valid = true;   // TODO: false until calibration is confirmed

        portENTER_CRITICAL(&s_mux);
        s_pose = p;
        portEXIT_CRITICAL(&s_mux);

        esp_task_wdt_reset();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(POSE_PERIOD_MS));
    }
}
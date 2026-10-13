#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "ModeManager.h"
#include "comms_task.h"
#include "control_task.h"
#include "arm_task.h"
#include "events_task.h"
#include "pose_task.h"        // ADDED
#include "autonomy_task.h"    // ADDED
#include "log_task.h"         // ADDED

robot::ModeManager g_modeManager;    // the only definition; booted in ESTOP

void comms_fill_telemetry(robot::proto::Telemetry& t) {
    const WheelState w = controlGetWheelState();
    t.pose.v = (w.speedL + w.speedR) * 0.5f;
    t.pose.w = (w.speedR - w.speedL) / TRACK_WIDTH_M;
    t.pose.valid = false;            // TODO: x, y, theta, valid from B's pose task
    t.arm       = armGetStatus();
    t.autoState = "IDLE";            // TODO: from C's autonomy
    t.battV     = NAN;               // TODO: ADC1 pin + voltage divider
}

void setup() {
    Serial.begin(115200);

    // Queues and hardware must be up before any task that touches them
    // starts. The scheduler doesn't run tasks until setup() returns.
    commsInit();      // creates queues
    armInit();        // servos; before control (control aborts arm on boot)
    controlInit();    // motors, encoders, PIDs
    poseInit();       // ADDED: IMU, odometry, fusion
    autonomyInit();   // ADDED: mission state machine
    logInit();        // ADDED: logger ring buffer

    // Higher number = higher priority.
    xTaskCreatePinnedToCore(controlTask,  "control",  4096, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(poseTask,     "pose",     4096, nullptr, 5, nullptr, 1);   // ADDED
    xTaskCreatePinnedToCore(armTask,      "arm",      4096, nullptr, 3, nullptr, 1);
    xTaskCreatePinnedToCore(autonomyTask, "autonomy", 4096, nullptr, 2, nullptr, 1);   // ADDED
    xTaskCreatePinnedToCore(eventsTask,   "events",   6144, nullptr, 2, nullptr, 1);
    xTaskCreatePinnedToCore(commsTask,    "comms",    8192, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(logTask,      "log",      2048, nullptr, 1, nullptr, 0);   // ADDED
}

void loop() { vTaskDelete(nullptr); }    // everything runs in the tasks above
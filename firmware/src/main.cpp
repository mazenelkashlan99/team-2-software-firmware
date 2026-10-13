#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "ModeManager.h"
#include "comms_task.h"
#include "control_task.h"
#include "arm_task.h"
#include "events_task.h"

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
    commsInit();
    controlInit();
    armInit();

    // Higher number = higher priority.
    xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(armTask,     "arm",     4096, nullptr, 3, nullptr, 1);
    xTaskCreatePinnedToCore(eventsTask,  "events",  6144, nullptr, 2, nullptr, 1);
    // TODO B: pose task    (core 1, priority 5)
    // TODO C: autonomy task (core 1, priority 2)
    xTaskCreatePinnedToCore(commsTask,   "comms",   8192, nullptr, 2, nullptr, 0);
}

void loop() { vTaskDelete(nullptr); }    // everything runs in the tasks above
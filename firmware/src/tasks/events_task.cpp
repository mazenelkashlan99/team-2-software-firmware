// events_task.cpp
#include "events_task.h"
#include <math.h>
#include "comms_task.h"
#include "control_task.h"
#include "arm_task.h"

using namespace robot;

namespace {

ModeGuards makeGuards() {
    ModeGuards g{};
    const WheelState w = controlGetWheelState();
    g.linkOk       = commsLinkOk();
    g.robotStopped = fabsf(w.speedL) < 0.02f && fabsf(w.speedR) < 0.02f;
    g.poseValid    = false;                         // TODO: from B's pose task
    g.armIdle      = armGetStatus() != ArmStatus::BUSY;
    return g;
}

void handle(const proto::Message& m) {
    switch (m.type) {
    case proto::MsgType::MODE: {
        TransitionResult r = g_modeManager.requestMode(m.mode, makeGuards(), EstopReason::NONE);
        Serial.printf("mode request: %s\n", ModeManager::resultName(r));
        break;
    }
    case proto::MsgType::ESTOP_RESET: {
        TransitionResult r = g_modeManager.requestMode(Mode::MANUAL, makeGuards(), EstopReason::NONE);
        Serial.printf("estop reset: %s\n", ModeManager::resultName(r));
        break;
    }
    case proto::MsgType::ARM:
        if (g_modeManager.current() != Mode::ESTOP && !armPost(m.arm))
            Serial.println("arm command rejected (busy)");
        break;
    case proto::MsgType::POSE_SET:
        // TODO B: poseReset(m.poseX, m.poseY, m.poseTheta);
        break;
    default:
        // START, GOTO, MISSION, CV_QR, CV_COLOR: C's autonomy task
        break;
    }
}

}  // namespace

void eventsTask(void*) {
    proto::Message msg;       // large (it holds the mission), so it lives in the task stack: 6 KB
    for (;;) {
        if (xQueueReceive(g_eventQ, &msg, portMAX_DELAY) == pdTRUE)
            handle(msg);
    }
}
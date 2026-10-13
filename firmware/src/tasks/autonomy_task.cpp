#include "autonomy_task.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "config.h"
#include "comms_task.h"        // g_modeManager
#include "control_task.h"      // controlPostAutoCommand()
#include "pose_task.h"         // poseGet()
#include "arm_task.h"          // armRequest*(), armGetStatus()
#include "Autonomy.h"          // state machine (pure C++)
#include "GoTo.h"              // (x,y) controller (pure C++)

namespace {

Autonomy s_autonomy;
GoTo     s_goto;

QueueHandle_t s_qrQ      = nullptr;
QueueHandle_t s_missionQ = nullptr;

void postMode(robot::Mode m) {
    g_modeManager.request(m);   // adjust to your ModeManager API
}

}  // namespace

void autonomyInit() {
    s_autonomy.begin();
    s_goto.begin();
    s_qrQ      = xQueueCreate(1, sizeof(robot::QrResult));
    s_missionQ = xQueueCreate(1, sizeof(robot::MissionConfig));
}

void autonomyPostQr(const robot::QrResult& qr) {
    xQueueOverwrite(s_qrQ, &qr);
}

void autonomyPostMission(const robot::MissionConfig& m) {
    xQueueOverwrite(s_missionQ, &m);
}

void autonomyTask(void*) {
    TickType_t wake = xTaskGetTickCount();

    robot::QrResult      qr {};
    robot::MissionConfig mission {};

    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(AUTONOMY_PERIOD_MS));

        // Snapshot newest external inputs. Non-blocking: use previous
        // value if nothing new arrived.
        xQueuePeek(s_qrQ,      &qr,      0);
        xQueuePeek(s_missionQ, &mission, 0);

        const robot::Mode mode = g_modeManager.current();
        if (mode != robot::Mode::SEMI && mode != robot::Mode::AUTO) {
            continue;
        }

        const robot::Pose pose = poseGet();
        if (!pose.valid) {
            controlPostAutoCommand(0.0f, 0.0f);   // refuse to act on bad pose
            continue;
        }

        const robot::ArmStatus arm = armGetStatus();

        // Step the state machine. Output is a small action + args —
        // policy lives in Autonomy, side effects live here.
        const AutonomyStep step =
            s_autonomy.step(pose, arm, qr, mission);

        switch (step.action) {
            case AutoAction::DriveTo: {
                const VelCmd v = s_goto.compute(pose, step.target, AUTONOMY_DT);
                controlPostAutoCommand(v.v, v.w);
                break;
            }
            case AutoAction::Stop:
                controlPostAutoCommand(0.0f, 0.0f);
                break;

            case AutoAction::ArmPick:
                armRequestPick();
                break;
            case AutoAction::ArmDrop:
                armRequestDrop();
                break;
            case AutoAction::ArmStow:
                armRequestStow();
                break;

            case AutoAction::RequestMode:
                postMode(step.nextMode);
                break;

            case AutoAction::Idle:
            case AutoAction::Done:
                break;
        }
    }
}
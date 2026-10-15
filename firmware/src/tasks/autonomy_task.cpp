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

}  // namespace

void autonomyInit() {
    s_autonomy.begin();
    s_goto.begin(1.0f, 2.0f, MAX_V_MPS, MAX_W_RADPS, 0.05f);
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
        const Autonomy::Step step =
            s_autonomy.step(pose, arm, qr, mission);

        switch (step.action) {
            case Autonomy::Step::Action::DriveTo: {
                const GoTo::Vel v = s_goto.compute(pose, step.targetX, step.targetY,
                                                   1.0f / AUTONOMY_RATE_HZ);
                controlPostAutoCommand(v.v, v.w);
                break;
            }
            case Autonomy::Step::Action::Stop:
                controlPostAutoCommand(0.0f, 0.0f);
                break;

            case Autonomy::Step::Action::ArmPick:
                armPost(robot::ArmCmd::PICK);
                break;
            case Autonomy::Step::Action::ArmDrop:
                armPost(robot::ArmCmd::DROP);
                break;
            case Autonomy::Step::Action::ArmStow:
                armPost(robot::ArmCmd::STOW);
                break;

            case Autonomy::Step::Action::Idle:
            case Autonomy::Step::Action::Done:
                break;
        }
    }
}

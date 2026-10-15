#include "arm_task.h"
#include <Arduino.h>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "config.h"
#include "ServoConfig.h"
#include "Arm.h"

namespace {

// NOTE: ARM_PERIOD_MS is defined in config.h — do NOT redefine it here.

Arm                  s_arm;
constexpr Arm::ServoConfig SERVO_CONFIG[] = {
    {PIN_SHOULDER, CH_SHOULDER, 500, 2500, 90.0f, 60.0f},
    {PIN_REACH,    CH_REACH,    500, 2500, 90.0f, 60.0f},
    {PIN_GRIPPER,  CH_GRIPPER,  500, 2500, 90.0f, 90.0f},
};
QueueHandle_t        s_cmdQ = nullptr;
std::atomic<bool>    s_abortReq{false};
std::atomic<bool>    s_ackReq{false};
std::atomic<uint8_t> s_status{0};

}  // namespace

void armInit() {
    s_cmdQ = xQueueCreate(1, sizeof(robot::ArmCmd));
    if (s_cmdQ == nullptr) return;

    s_arm.begin(SERVO_CONFIG, SERVO_COUNT);

    // Optional, once the IR sensor exists:
    // s_arm.setHoldCheck([]() -> bool { return irGripper.detected(); });

    s_status.store(static_cast<uint8_t>(s_arm.status()));
}

bool armPost(robot::ArmCmd cmd) {
    if (s_cmdQ == nullptr) return false;
    return xQueueSend(s_cmdQ, &cmd, 0) == pdTRUE;
}

void armRequestAbort()          { s_abortReq.store(true); }
void armRequestAck()            { s_ackReq.store(true); }

robot::ArmStatus armGetStatus() {
    return static_cast<robot::ArmStatus>(s_status.load());
}

void armTask(void*) {
    TickType_t wake = xTaskGetTickCount();

    for (;;) {
        // Abort: drop any pending command, return the arm to idle.
        if (s_abortReq.exchange(false)) {
            s_arm.abort();
            xQueueReset(s_cmdQ);
        }

        // Ack: tell the arm the last DONE/FAILED was seen.
        if (s_ackReq.exchange(false)) {
            s_arm.acknowledge();
        }

        // New command? Only start one when the arm isn't mid-motion.
        robot::ArmCmd cmd;
        if (!s_arm.busy() && xQueueReceive(s_cmdQ, &cmd, 0) == pdTRUE) {
            s_arm.start(cmd);
        }

        // Advance the servo ramp by one tick.
        s_arm.update(ARM_TICK_MS / 1000.0f);

        // Publish status for polled consumers.
        s_status.store(static_cast<uint8_t>(s_arm.status()));

        vTaskDelayUntil(&wake, pdMS_TO_TICKS(ARM_TICK_MS));
    }
}

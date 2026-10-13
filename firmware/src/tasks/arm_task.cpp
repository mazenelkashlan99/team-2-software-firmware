#include "arm_task.h"
#include <Arduino.h>
#include <atomic>
#include "config.h"
#include "ServoConfig.h"
#include "Arm.h"

namespace {
Arm               s_arm;
QueueHandle_t     s_cmdQ = nullptr;
std::atomic<bool> s_abortReq{false};
std::atomic<bool> s_ackReq{false};
std::atomic<uint8_t> s_status{0};
}

void armInit() {
    s_cmdQ = xQueueCreate(1, sizeof(robot::ArmCmd));
    s_arm.begin(SERVO_CONFIG, SERVO_COUNT);
    // Optional, once the IR sensor exists:
    // s_arm.setHoldCheck([]() -> bool { return irGripper.detected(); });
    s_status.store((uint8_t)s_arm.status());
}

bool armPost(robot::ArmCmd cmd)   { return xQueueSend(s_cmdQ, &cmd, 0) == pdTRUE; }
void armRequestAbort()            { s_abortReq.store(true); }
void armRequestAck()              { s_ackReq.store(true); }
robot::ArmStatus armGetStatus()   { return (robot::ArmStatus)s_status.load(); }

void armTask(void*) {
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        // Only this task touches the Arm object: other tasks set flags or post to the queue.
        if (s_abortReq.exchange(false)) {
            s_arm.abort();
            xQueueReset(s_cmdQ);                 // a pending command must not run after an e-stop
        }
        if (s_ackReq.exchange(false)) s_arm.acknowledge();

        robot::ArmCmd cmd;
        if (!s_arm.busy() && xQueueReceive(s_cmdQ, &cmd, 0) == pdTRUE)
            s_arm.start(cmd);

        s_arm.update(ARM_PERIOD_MS / 1000.0f);
        s_status.store((uint8_t)s_arm.status());
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(ARM_PERIOD_MS));
    }
}
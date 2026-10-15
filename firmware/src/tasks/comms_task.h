#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "robot_interfaces.h"
#include "ModeManager.h"
#include "Protocol.h"

struct DriveCmd {
    float thr = 0.0f;
    float turn = 0.0f;
};

extern QueueHandle_t     g_driveQ;
extern QueueHandle_t     g_eventQ;
extern robot::ModeManager g_modeManager;

void commsInit();
void commsTask(void* arg);
bool commsLinkOk();
void comms_fill_telemetry(robot::proto::Telemetry& t);

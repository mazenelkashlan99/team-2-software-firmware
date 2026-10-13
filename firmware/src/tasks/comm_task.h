#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "Protocol.h"
#include "ModeManager.h"

struct DriveCmd {
    float    thr;    // -1..1
    float    turn;   // -1..1
    uint32_t rxMs;   // millis() when received
};

extern QueueHandle_t g_driveQ;   // length 1, always holds only the NEWEST drive command
extern QueueHandle_t g_eventQ;   // robot::proto::Message: MODE, ESTOP_RESET, START, GOTO,
                                 // ARM, POSE_SET, MISSION, CV_QR, CV_COLOR
extern robot::ModeManager g_modeManager;   // defined once, in main.cpp

void commsInit();            // creates the queues; call before starting the task
void commsTask(void* arg);   // FreeRTOS task, pin it to core 0
bool commsLinkOk();          // true while the console heartbeat is fresh

// Implemented by main.cpp: fills pose, arm, autoState and battV.
// (mode and linkOk are filled here.)
void comms_fill_telemetry(robot::proto::Telemetry& t);
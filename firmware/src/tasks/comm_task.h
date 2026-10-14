#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "robot_interfaces.h"

// WiFi + MQTT link to the console.
//
// Owns g_driveQ and g_eventQ. The task receives MQTT traffic, decodes
// via Protocol, and pushes to those queues. Producers (main, autonomy)
// read from them.

// Creates the queues. Call from setup() before commsTask() runs.
void commsInit();

// FreeRTOS task entry. Blocking loop; never returns.
void commsTask(void* arg);

// Cross-task read: is the console link alive?
// True while the broker is connected AND a console heartbeat arrived
// within the link timeout. Read by control_task for mode gating.
bool commsLinkOk();

// Latest drive command, overwrite slot. Producer: onMqttMessage.
// Consumer: control_task. Reading does not consume (peek semantics).
extern QueueHandle_t g_driveQ;

// Event queue: decoded messages that aren't DRIVE or HEARTBEAT.
// Consumer: events_task. Bounded at 8; drops when full.
extern QueueHandle_t g_eventQ;

// Fills the telemetry struct from current system state.
// Implemented in main.cpp. Called by commsTask at 10 Hz.
void comms_fill_telemetry(robot::proto::Telemetry& t);
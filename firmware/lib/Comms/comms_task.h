// comms_task.h -- ESP32 glue: MQTTManager <-> CommsCore (implementation only
// compiled on the ESP32, see comms_task.cpp).
#pragma once

#include "comms_core.h"

namespace robot {

// Starts the MQTT task and the TX task. Returns false if memory is missing.
// modes, ports and src must outlive the tasks (use statics).
// Does not wait for Wi-Fi: the robot stays in ESTOP until the console
// sends robot/cmd/estop_reset.
bool startComms(ModeManager& modes, const CommsPorts& ports,
                const CommsSources& src,
                const CommsConfig& cfg = CommsConfig());

// For Person A's control task:  comms().getCommand(millis())
CommsCore& comms();

}  // namespace robot

#pragma once
#include "robot_interfaces.h"

void armInit();
void armTask(void* arg);

// Non-blocking: returns true if the command was queued, false if the
// queue is full. Only armTask touches the Arm object itself.
bool armPost(robot::ArmCmd cmd);

// Async requests. Both set a flag; armTask checks and clears them.
void armRequestAbort();   // cancel current motion, drop pending command
void armRequestAck();     // acknowledge a DONE/FAILED status

robot::ArmStatus armGetStatus();
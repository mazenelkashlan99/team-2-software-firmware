#pragma once
#include "robot_interfaces.h"     // robot::ArmCmd, robot::ArmStatus

void armInit();                       // blocks a few hundred ms (staggered servo startup)
void armTask(void*);
bool armPost(robot::ArmCmd cmd);      // false if a command is already waiting
void armRequestAbort();               // e-stop path: servos hold their angle
void armRequestAck();                 // clear DONE / FAILED back to IDLE
robot::ArmStatus armGetStatus();
#pragma once
#include "robot_interfaces.h"
void armInit();
void armTask(void* arg);
robot::ArmStatus armGetStatus();
void armRequestAbort();
void armRequestPick();
void armRequestDrop();
void armRequestStow();

#pragma once
#include "robot_interfaces.h"
void poseInit();
void poseTask(void* arg);
robot::Pose poseGet();

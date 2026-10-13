#pragma once
#include <stdint.h>
#include "WheelController.h"

struct PoseData {
  float x, y, theta;   
  float v, omega;      
  uint32_t tMs;        
};

void poseTaskStart(WheelController& left, WheelController& right, float trackWidthM);

bool getPose(PoseData& out);
void resetPose(float x, float y, float theta);
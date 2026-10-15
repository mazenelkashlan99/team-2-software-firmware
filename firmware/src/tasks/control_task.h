#pragma once
#include <stdint.h>

struct WheelState {
    float speedL = 0.0f;
    float speedR = 0.0f;
    float totalL = 0.0f;
    float totalR = 0.0f;
    uint32_t tick = 0;
};

void controlInit();
void controlTask(void* arg);
WheelState controlGetWheelState();
void controlPostAutoCommand(float v, float w);
float wheelDistanceDeltaLeft();
float wheelDistanceDeltaRight();

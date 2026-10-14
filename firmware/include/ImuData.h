#pragma once
#include <stdint.h>

// Plain data struct. No hardware dependencies, so it can be included
// from pure-C++ code (Fusion, tests) without dragging in Wire.h.
struct ImuData {
    float ax = 0.0f;    // m/s^2
    float ay = 0.0f;
    float az = 0.0f;
    float gx = 0.0f;    // rad/s
    float gy = 0.0f;
    float gz = 0.0f;    // rad/s, after bias removal
    float tempC = 0.0f;
    uint32_t tUs = 0;   // micros() at sample time
};
#pragma once
#include "robot_interfaces.h"

// Proportional controller toward a target (x, y). Returns {v, w} in
// SI units. Pure C++; no hardware.
//
// Does not avoid obstacles, plan paths, or enforce safety limits.
// Those are the caller's job.
class GoTo {
public:
    struct Vel { float v = 0.0f; float w = 0.0f; };

    void begin(float kRho, float kAlpha, float vMax, float wMax, float tolerance);

    // dt is currently unused; kept for future derivative / feed-forward.
    Vel  compute(const robot::Pose& pose, float targetX, float targetY, float dt);

    bool atTarget(const robot::Pose& pose, float targetX, float targetY) const;

private:
    float kRho_      = 1.0f;
    float kAlpha_    = 2.0f;
    float vMax_      = 0.5f;
    float wMax_      = 1.5f;
    float tolerance_ = 0.05f;
};
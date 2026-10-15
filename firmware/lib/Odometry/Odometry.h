#pragma once
#include "robot_interfaces.h"

// Differential-drive odometry. Integrates per-wheel distance deltas
// into a pose. Does NOT measure speed; v/w come from elsewhere.
//
// Frame (matches robot_interfaces.h Pose):
//   x: m, +x to the right of start
//   y: m, +y toward the box row
//   theta: rad, 0 along +x, +CCW, wrapped to (-pi, pi]
class Odometry {
public:
    void begin(float trackWidth);

    // Signed meters traveled since the previous call.
    void update(float dLeft, float dRight);

    robot::Pose pose() const { return pose_; }

    void resetPose(float x, float y, float theta);

private:
    static float wrapPi(float t);

    float       trackWidth_ = 0.15f;
    robot::Pose pose_ {};
};
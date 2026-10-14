#pragma once
#include "robot_interfaces.h"
#include "ImuData.h"      // was "Imu.h" — now points to include/ImuData.h

// Complementary filter. Fast yaw from the gyro, drift-corrected by
// odometry-derived yaw. x/y pass straight through from odometry.
//
// Caller must supply the sample period. If your ImuData carries a
// timestamp, prefer a `setDt()` updated from consecutive tUs deltas.
class Fusion {
public:
    void begin(float alpha);
    void setDt(float dt);

    // Returns a fused pose. odom.x/y are trusted as-is; only theta is
    // filtered. Set odom.valid to false to have fusion pass through.
    robot::Pose update(const robot::Pose& odom, const ImuData& imu);

private:
    float alpha_ = 0.98f;
    float dt_    = 0.01f;
    float theta_ = 0.0f;
    bool  first_ = true;
};
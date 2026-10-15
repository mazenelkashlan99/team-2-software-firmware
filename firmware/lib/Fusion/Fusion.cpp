#include "Fusion.h"

void Fusion::begin(float alpha) {
    if (alpha > 0.0f && alpha < 1.0f) alpha_ = alpha;
    theta_ = 0.0f;
    first_ = true;
}

void Fusion::setDt(float dt) {
    if (dt > 0.0f) dt_ = dt;
}

robot::Pose Fusion::update(const robot::Pose& odom, const ImuData& imu) {
    if (first_) {
        theta_ = odom.theta;
        first_ = false;
    } else {
        const float gyroYaw = theta_ + imu.gz * dt_;
        theta_ = alpha_ * gyroYaw + (1.0f - alpha_) * odom.theta;
    }

    robot::Pose out = odom;
    out.theta = theta_;
    return out;
}
#include "Odometry.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void Odometry::begin(float trackWidth) {
    if (trackWidth > 0.0f) trackWidth_ = trackWidth;
    resetPose(0.0f, 0.0f, 0.0f);
}

void Odometry::resetPose(float x, float y, float theta) {
    pose_.x     = x;
    pose_.y     = y;
    pose_.theta = wrapPi(theta);
    pose_.v     = 0.0f;
    pose_.w     = 0.0f;
    pose_.t     = 0;
    pose_.valid = true;
}

float Odometry::wrapPi(float t) {
    while (t >  static_cast<float>(M_PI)) t -= 2.0f * static_cast<float>(M_PI);
    while (t < -static_cast<float>(M_PI)) t += 2.0f * static_cast<float>(M_PI);
    return t;
}

void Odometry::update(float dLeft, float dRight) {
    const float dCenter = 0.5f * (dLeft + dRight);
    const float dTheta  = (dRight - dLeft) / trackWidth_;

    // Midpoint integration: use the average heading over the tick.
    const float thetaMid = pose_.theta + 0.5f * dTheta;

    pose_.x += dCenter * std::cos(thetaMid);
    pose_.y += dCenter * std::sin(thetaMid);
    pose_.theta = wrapPi(pose_.theta + dTheta);
}
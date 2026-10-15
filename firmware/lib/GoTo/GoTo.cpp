#include "GoTo.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void GoTo::begin(float kRho, float kAlpha, float vMax, float wMax, float tol) {
    kRho_      = kRho;
    kAlpha_    = kAlpha;
    vMax_      = vMax;
    wMax_      = wMax;
    tolerance_ = tol;
}

GoTo::Vel GoTo::compute(const robot::Pose& pose, float tx, float ty, float /*dt*/) {
    const float dx = tx - pose.x;
    const float dy = ty - pose.y;

    const float rho = std::sqrt(dx * dx + dy * dy);
    if (rho < tolerance_) return {0.0f, 0.0f};

    const float angleToTarget = std::atan2(dy, dx);
    float alpha = angleToTarget - pose.theta;
    while (alpha >  static_cast<float>(M_PI)) alpha -= 2.0f * static_cast<float>(M_PI);
    while (alpha < -static_cast<float>(M_PI)) alpha += 2.0f * static_cast<float>(M_PI);

    // Slow down when the robot isn't facing the target; turn toward it.
    float v = kRho_   * rho * std::cos(alpha);
    float w = kAlpha_ * alpha;

    if (v >  vMax_) v =  vMax_;
    if (v < -vMax_) v = -vMax_;
    if (w >  wMax_) w =  wMax_;
    if (w < -wMax_) w = -wMax_;

    return {v, w};
}

bool GoTo::atTarget(const robot::Pose& pose, float tx, float ty) const {
    const float dx = tx - pose.x;
    const float dy = ty - pose.y;
    return (dx * dx + dy * dy) < (tolerance_ * tolerance_);
}
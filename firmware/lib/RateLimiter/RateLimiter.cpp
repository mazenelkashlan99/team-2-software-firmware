#include "RateLimiter.h"

void RateLimiter::setMaxRate(float unitsPerSec) {
    if (unitsPerSec >= 0.0f) maxRate_ = unitsPerSec;
}

void RateLimiter::reset(float value) {
    value_ = value;
}

float RateLimiter::update(float target, float dt) {
    if (dt <= 0.0f) return value_;

    if (maxRate_ == 0.0f) {
        value_ = target;
        return value_;
    }

    const float maxStep = maxRate_ * dt;
    const float diff    = target - value_;

    if (diff >  maxStep) value_ += maxStep;
    else if (diff < -maxStep) value_ -= maxStep;
    else value_ = target;

    return value_;
}
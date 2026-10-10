#include "Ramp.h"

// Goal: store the rate limit.
void Ramp::setMaxRate(float maxRatePerSec) {

    if (maxRatePerSec >= 0){
        maxRate_=maxRatePerSec;
    }

}

// Goal: move value_ toward target, but never faster than the rate allows.
float Ramp::update(float target, float dt) {
    if (dt <= 0.0f) {
        return value_;        
    }

    // maxRate_ == 0 means "no limit, follow target"
    if (maxRate_ == 0.0f) {
        value_ = target;
        return value_;
    }

    const float maxStep = maxRate_ * dt;
    const float diff    = target - value_;

    if (diff >  maxStep) value_ += maxStep;
    else if (diff < -maxStep) value_ -= maxStep;
    else value_ = target;   // within one step: snap

    return value_;

}

// Goal: jump the output to a value.
void Ramp::reset(float value) {
    value_ = value;
}
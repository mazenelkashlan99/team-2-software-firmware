#include "PID.h"
#include <stdlib.h>

namespace {
}

// Goal: store the gains.
void PID::setGains(float kp, float ki, float kd, float kff) {
    kp_=kp;
    ki_=ki;
    kd_=kd;
    kff_=kff;
}

// Goal: store the output limits.
void PID::setOutputLimits(float outMin, float outMax) {
    if (outMin>1){
        outMin=1;
    }
    if (outMin<-1){
        outMin=-1;
    }
    if (outMax>1){
        outMax=1;
    }
    if (outMax<-1){
        outMax=-1;
    }

    if (outMin > outMax) {
        float tmp = outMin;
        outMin = outMax;
        outMax = tmp;
    }

    outMin_=outMin;
    outMax_=outMax;
}

// Goal: store the integral limits.
void PID::setIntegralLimits(float intMin, float intMax) {

    if (intMin > intMax) {
        float tmp = intMin;
        intMin = intMax;
        intMax = tmp;
    }

    intMin_=intMin;
    intMax_=intMax;
}

// Goal: turn a setpoint and a measurement into one power value in [outMin_, outMax_].
float PID::compute(float setpoint, float measurement, float dt) {
    // 1. Guard against bad dt — don't touch state.
    if (dt <= 0.0f) {
        float ff = kff_ * setpoint;
        if (ff > outMax_) ff = outMax_;
        if (ff < outMin_) ff = outMin_;
        return ff;
    }

    // 2. Error.
    const float error = setpoint - measurement;

    // 3. Derivative on measurement (skip on first call).
    float derivative = 0.0f;
    if (!firstRun_) {
        derivative = (measurement - prevMeasurement_) / dt;
    }
    prevMeasurement_ = measurement;
    firstRun_ = false;

    // 4. Integral, accumulated in power units, clamped (anti-windup).
    integral_ += ki_ * error * dt;
    if (integral_ > intMax_) integral_ = intMax_;
    if (integral_ < intMin_) integral_ = intMin_;

    // 5. Sum: FF + P + I - D(measurement).
    float output = kff_ * setpoint
                 + kp_  * error
                 + integral_
                 - kd_  * derivative;

    // 6. Clamp before returning.
    if (output > outMax_) output = outMax_;
    if (output < outMin_) output = outMin_;
    return output;
}

// Goal: forget the history.
void PID::reset() {
    integral_ = 0.0f; 
    prevMeasurement_ = 0.0f;   
    firstRun_ = true;   
}
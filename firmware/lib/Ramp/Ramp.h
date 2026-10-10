#pragma once

// Rate limiter for one value (used for v in m/s and w in rad/s).
//
// Pure C++: no Arduino.h, no hardware, dt passed in. Testable on the laptop.
//
// The output moves toward the target by at most maxRate * dt per call.
// A step in the target therefore becomes a ramp, and a speed profile of
// "ramp up, cruise, ramp down" is the trapezoid.
//
// Same idea as Motor::setSlewRate(), but on a velocity (units per second)
// instead of on motor power.

class Ramp {
public:
    Ramp() = default;

    // Max change per second (e.g. 1.0 m/s^2 for v, 4.0 rad/s^2 for w).
    // 0 = ramp off: the output follows the target instantly.
    // Negative values are ignored.
    void setMaxRate(float maxRatePerSec);

    // Move the output toward target by at most maxRate * dt, and return it.
    //   target : desired value
    //   dt     : seconds since the previous call
    // dt <= 0 leaves the output unchanged.
    float update(float target, float dt);

    // Jump the output to a value without ramping (e.g. 0 after a stop).
    void reset(float value = 0.0f);

    float value() const       { return value_; }
    float getMaxRate() const  { return maxRate_; }

private:
    float maxRate_ = 0.0f;
    float value_   = 0.0f;
};
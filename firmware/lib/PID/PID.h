#pragma once

// Velocity PID controller for one wheel.
//
// Pure C++: no Arduino.h, no hardware calls, no millis(). dt is passed in
// by the caller, so the class is unit-testable on the laptop (env:native).
//
// Features required by the firmware plan:
//   - Feed-forward term (kff * setpoint), which does most of the work.
//   - Anti-windup: the integral term is clamped, so it cannot grow
//     unbounded while the output is saturated (e.g. during a stall or
//     while the motor is at max power).
//   - Derivative on measurement, not on error, so a step change in the
//     setpoint does not produce a "derivative kick".
//
// Units: pick one and stay consistent across the whole project.
// The project convention is SI: setpoint and measurement in m/s,
// output in the range [-1, 1] to feed Motor::setPower().

class PID {
public:
    PID() = default;

    // ---- Configuration --------------------------------------------

    // kp  : proportional gain
    // ki  : integral gain
    // kd  : derivative gain
    // kff : feed-forward gain (power per unit of setpoint)
    void setGains(float kp, float ki, float kd, float kff);

    // Clamp the output. Defaults match Motor::setPower(): [-1, 1].
    void setOutputLimits(float outMin, float outMax);

    // Clamp the integral accumulator. Usually the same as the output
    // limits; tighter limits reduce overshoot, looser limits let the
    // controller fight steady-state error harder.
    void setIntegralLimits(float intMin, float intMax);

    // ---- Runtime ---------------------------------------------------

    // Compute the next output. Call at a fixed rate.
    //   setpoint    : desired value (e.g. wheel speed in m/s)
    //   measurement : measured value (e.g. encoder speed in m/s)
    //   dt          : seconds since the previous call
    // Returns a value clamped to [outMin_, outMax_].
    float compute(float setpoint, float measurement, float dt);

    // Zero the integral and derivative state. Call when the controller
    // is re-enabled after a stop, or when the setpoint changes mode.
    void reset();

    // ---- Introspection --------------------------------------------

    float getKp()  const { return kp_;  }
    float getKi()  const { return ki_;  }
    float getKd()  const { return kd_;  }
    float getKff() const { return kff_; }

private:
    // Gains
    float kp_  = 0.0f;
    float ki_  = 0.0f;
    float kd_  = 0.0f;
    float kff_ = 0.0f;

    // Output and integral clamps
    float outMin_ = -1.0f;
    float outMax_ =  1.0f;
    float intMin_ = -1.0f;
    float intMax_ =  1.0f;

    // State
    float integral_        = 0.0f;   // accumulated error * dt
    float prevMeasurement_ = 0.0f;   // for derivative-on-measurement
    bool  firstRun_        = true;   // skip D on the first call
};
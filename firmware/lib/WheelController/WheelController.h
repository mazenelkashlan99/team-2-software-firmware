#pragma once
#include <stdint.h>
#include "Motor-Controller.h"
#include "Encoder.h"
#include "PID.h"

// Speed control loop for ONE wheel:
//
//   Encoder ticks -> speed (m/s, filtered) -> PID -> Motor::setPower()
//
// Does not own the Motor, Encoder or PID: it keeps references, so the objects
// must outlive it (create them as globals or in main).
//
// It is also the ONLY reader of the encoder. Odometry (Person B) reads the
// wheel distance from here and never touches the Encoder directly, because
// two readers of one encoder would confuse each other's deltas.
//
// Units: meters, m/s, seconds. Positive speed = robot moves forward.

class WheelController {
public:
    // ticksPerMeter : encoder ticks per meter of wheel travel (from config.h)
    WheelController(MotorController & motor, Encoder& encoder, PID& pid, float ticksPerMeter);

    // ---- Configuration -----------------------------------------------

    // If the count goes DOWN when the wheel moves forward, set true.
    // Fixes the encoder sign in one place instead of in the PID.
    void setInverted(bool inverted);

    // Low-pass filter on the measured speed. alpha in (0, 1]:
    // 1 = no filtering, smaller = smoother but slower to react.
    void setSpeedFilter(float alpha);

    // ---- Runtime -----------------------------------------------------

    // Desired wheel speed in m/s (negative = backward).
    void setTargetSpeed(float mps);

    // Call at a fixed rate (e.g. 100 Hz) with dt in seconds.
    // Reads the encoder, updates speed and distance, runs the PID and
    // applies the result to the motor (including Motor::update(dt)).
    void update(float dt);

    // Target = 0, PID history cleared, motor coasts / brakes.
    void stop();
    void brake();

    // ---- Readings (for DriveBase and for B's odometry) ----------------

    void setHold(bool hold) { hold_ = hold; }
    void reset(float value);
    float getTargetSpeed()   const { return target_; }
    float getSpeed()         const { return speed_; }        // m/s, filtered
    float getDistanceDelta() const { return distDelta_; }    // m moved in the last update()
    float getTotalDistance() const { return totalDist_; }    // m since start (signed)

private:
    MotorController&   motor_;
    Encoder& encoder_;
    PID&     pid_;
    float    ticksPerMeter_;
    bool     inverted_ = false;
    bool     hold_ = false;

    float target_    = 0.0f;   // m/s requested
    float speed_     = 0.0f;   // m/s measured, filtered
    float distDelta_ = 0.0f;   // m in the last tick
    float totalDist_ = 0.0f;   // m accumulated
    float alpha_     = 0.3f;   // speed filter

    int32_t lastTicks_ = 0;
    bool    firstRun_  = true; // first update only records ticks, no delta
};
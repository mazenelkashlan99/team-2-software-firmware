#pragma once
#include "WheelController.h"
#include "Ramp.h"

// Differential-drive base: turns a {v, w} command into two wheel speeds.
//
//   vL = v - w * L / 2        vR = v + w * L / 2        (L = track width)
//
// Pipeline per tick:  command -> clamp -> Ramp (v and w) -> wheel speeds
//                     -> WheelController x2
//
// Failsafe: if no setCommand() arrives within the timeout, the command
// becomes {0, 0}. Time is counted from the dt passed to update(), so the
// class never calls millis().
//
// Units: m/s for v, rad/s for w (positive = counter-clockwise), meters for L.

class DriveBase {
public:
    DriveBase(WheelController& left, WheelController& right, float trackWidth);

    // ---- Configuration -----------------------------------------------

    // Seconds without a new command before the base stops itself.
    void setCommandTimeout(float seconds);

    // Max |v| (m/s) and |w| (rad/s) accepted. Keep
    // maxV + maxW * L / 2 below what the wheels can actually do.
    void setLimits(float maxV, float maxW);

    // Max acceleration for v (m/s^2) and for w (rad/s^2). 0 = ramp off.
    void setRamps(float accelV, float accelW);

    // ---- Runtime -----------------------------------------------------

    // New command from B (go-to), C (manual / autonomy) or the console.
    // Also restarts the timeout. Ignored while the emergency stop is latched.
    void setCommand(float v, float w);

    // Call at a fixed rate (e.g. 100 Hz) with dt in seconds.
    void update(float dt);

    // Normal stop: command {0, 0}, ramps down smoothly.
    void stop();

    // Immediate stop: bypasses the ramps, brakes both wheels, and ignores
    // commands until clearEmergencyStop() is called.
    void emergencyStop();
    void clearEmergencyStop();

    // ---- Readings ----------------------------------------------------

    float getCommandedV() const { return cmdV_; }    // after clamping, before the ramp
    float getCommandedW() const { return cmdW_; }
    bool  isTimedOut()    const { return timedOut_; }
    bool  isEmergencyStopped() const { return estop_; }

private:
    WheelController& left_;
    WheelController& right_;
    float trackWidth_;

    Ramp  rampV_;
    Ramp  rampW_;

    float cmdV_  = 0.0f;
    float cmdW_  = 0.0f;
    float maxV_  = 1.0f;     // placeholder limits, set from config.h
    float maxW_  = 3.0f;

    float timeoutS_     = 0.5f;
    float sinceCmdS_    = 0.0f;
    bool  timedOut_     = false;
    bool  estop_        = false;
};
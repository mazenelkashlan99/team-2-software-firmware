#include "DriveBase.h"
#include <cmath>


// References must be bound in the member-initializer list; they cannot
// be assigned later. Ramp members default-construct.
DriveBase::DriveBase(WheelController& left, WheelController& right, float trackWidth)
    : left_(left), right_(right), trackWidth_(trackWidth) {}


// ---- Configuration -----------------------------------------------

// Reject non-positive timeouts. A timeout of 0 would fire on the very
// first tick after any command, which is a caller bug, not an intent.
void DriveBase::setCommandTimeout(float seconds) {
    if (seconds > 0.0f) {
        timeoutS_ = seconds;
    }
}


// Limits are magnitudes. A limit of 0 would permanently lock the robot
// on that axis, so treat it as a bug and keep the old value.
void DriveBase::setLimits(float maxV, float maxW) {
    maxV = fabsf(maxV);
    maxW = fabsf(maxW);

    if (maxV > 0.0f) maxV_ = maxV;
    if (maxW > 0.0f) maxW_ = maxW;
}


// Ramp::setMaxRate already accepts 0 ("follow target immediately") and
// ignores negatives, so no extra validation is needed here.
void DriveBase::setRamps(float accelV, float accelW) {
    rampV_.setMaxRate(accelV);
    rampW_.setMaxRate(accelW);
}


// ---- Runtime -----------------------------------------------------

void DriveBase::setCommand(float v, float w) {
    // Latched emergency stop wins over any incoming command.
    if (estop_) return;

    // Clamp to configured limits. The caller doesn't get to know they
    // were clamped; getCommandedV/W() reports what was accepted.
    if (v >  maxV_) v =  maxV_;
    if (v < -maxV_) v = -maxV_;
    if (w >  maxW_) w =  maxW_;
    if (w < -maxW_) w = -maxW_;

    cmdV_ = v;
    cmdW_ = w;

    // A fresh command is a heartbeat: restart the timeout window.
    sinceCmdS_ = 0.0f;
    timedOut_  = false;
}


// Request a smooth stop. Do NOT reset the ramps — deceleration is the
// ramp's job. A stop is a valid command, so it also restarts the timer.
void DriveBase::stop() {
    cmdV_ = 0.0f;
    cmdW_ = 0.0f;

    sinceCmdS_ = 0.0f;
    timedOut_  = false;
}


// Latched emergency stop: a later setCommand() is ignored until
// clearEmergencyStop() is called.
void DriveBase::emergencyStop() {
    estop_ = true;

    cmdV_ = 0.0f;
    cmdW_ = 0.0f;

    // Reset ramp state so clearing the latch later starts from rest,
    // not from whatever speeds the ramps had before the e-stop.
    rampV_.reset(0.0f);
    rampW_.reset(0.0f);

    // Sync the wheel controllers to "stopped" and hold them there.
    // setHold keeps the encoder/odometry side of update() alive while
    // writing zero motor power, so distance deltas don't go stale.
    left_.reset(0.0f);
    right_.reset(0.0f);
    left_.setHold(true);
    right_.setHold(true);
}


// Release the latch. Command, ramps, and wheel speeds are already zero
// from emergencyStop(), so the robot stays still until a fresh command
// arrives — and that command ramps up from rest.
void DriveBase::clearEmergencyStop() {
    estop_ = false;

    sinceCmdS_ = 0.0f;
    timedOut_  = false;

    left_.setHold(false);
    right_.setHold(false);
}


// One tick of the drive loop. Order matters:
//   1. timeout bookkeeping
//   2. emergency latch   — hold wheels, skip ramps
//   3. timeout target    — zero target; ramps decelerate smoothly
//   4. ramps             — smooth v and w
//   5. mixing            — (v, w) -> (vL, vR)
//   6. wheel controllers — encoders, PID, motor
void DriveBase::update(float dt) {
    if (dt <= 0.0f) return;

    // 1. Timeout bookkeeping
    sinceCmdS_ += dt;
    if (timeoutS_ > 0.0f && sinceCmdS_ > timeoutS_) {
        timedOut_ = true;
    }

    // 2. Emergency latch. Wheels stay live for odometry, but their
    //    hold flag keeps motor power at 0; ramps are untouched.
    if (estop_) {
        left_.update(dt);
        right_.update(dt);
        return;
    }

    // 3. A timeout zeroes the *target*, not the actual speed — the
    //    ramps still decelerate smoothly rather than snapping to 0.
    if (timedOut_) {
        cmdV_ = 0.0f;
        cmdW_ = 0.0f;
    }

    // 4. Ramp v and w toward their targets at the configured rates.
    //    If a rate is 0, Ramp::update snaps to target (ramp off).
    const float rampedV = rampV_.update(cmdV_, dt);
    const float rampedW = rampW_.update(cmdW_, dt);

    // 5. Differential-drive mixing:
    //      vL = v - w * L/2
    //      vR = v + w * L/2
    const float half   = trackWidth_ * 0.5f;
    const float vLeft  = rampedV - rampedW * half;
    const float vRight = rampedV + rampedW * half;

    left_.setTargetSpeed(vLeft);
    right_.setTargetSpeed(vRight);

    // 6. Run each wheel controller's own tick.
    left_.update(dt);
    right_.update(dt);
}
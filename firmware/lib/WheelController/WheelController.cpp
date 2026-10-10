#include "WheelController.h"

// Q: the class holds REFERENCES (Motor&, Encoder&, PID&). A reference member can't be
//    assigned in the constructor body. Where must it be initialized?
WheelController::WheelController(MotorController & motor, Encoder& encoder, PID& pid, float ticksPerMeter) 
: motor_(motor), encoder_(encoder), pid_(pid), ticksPerMeter_(ticksPerMeter) {}

// Goal: fix the encoder sign in one place.
void WheelController::setInverted(bool inverted) {
    inverted_=inverted;
}

// Goal: set the speed filter strength.
// Q: what range is valid for alpha, and what should happen outside it?
void WheelController::setSpeedFilter(float alpha) {
    if (alpha > 0.0f && alpha <= 1.0f) {
        alpha_=alpha;
    }
}

// Goal: record the requested wheel speed in m/s.
void WheelController::setTargetSpeed(float mps) {
    target_=mps;
}

void WheelController::update(float dt) {
    // ---- 1. Guard the timestep --------------------------------------
    // dt is about to become a divisor. Zero or negative would produce
    // inf/NaN in step 5, and NaN survives clamping. Bail before
    // touching any state, so the next call gets a clean slate.
    if (dt <= 0.0f) return;

    // ---- 2. Read raw encoder ----------------------------------------
    // Signed 32-bit: a backward-spinning wheel produces a smaller
    // (or negative) count than last time, and we want that sign.
    int32_t ticks = encoder_.getTicks();

    // ---- 3. First call: no history yet ------------------------------
    // lastTicks_ is meaningless until we've seen a tick. Subtracting
    // it would treat "ticks since boot" as distance traveled this
    // tick. Just record where we are and get out — don't move the
    // motor on this call.
    if (firstRun_) {
        lastTicks_ = ticks;
        firstRun_ = false;
        distDelta_ = 0.0f;
        return;
    }

    // ---- 4. Ticks since the previous call ---------------------------
    // Subtract BEFORE overwriting lastTicks_, otherwise delta is 0.
    // int32_t subtraction wraps correctly on encoder rollover.
    int32_t deltaTicks = ticks - lastTicks_;
    lastTicks_ = ticks;

    // ---- 5. Ticks -> meters -----------------------------------------
    // Cast to float first: integer division would round 1/900 to 0,
    // and every small motion would vanish.
    float deltaMeters = static_cast<float>(deltaTicks) / ticksPerMeter_;

    // ---- 6. Normalize sign ------------------------------------------
    // Fix the "which way is forward" convention HERE, once, so every
    // consumer downstream (PID, telemetry, odometry) can assume
    // positive = forward.
    if (inverted_) deltaMeters = -deltaMeters;

    // ---- 7. Distance bookkeeping ------------------------------------
    distDelta_ = deltaMeters;    // transient: distance this tick
    totalDist_ += deltaMeters;   // persistent: distance since reset

    // ---- 8. Raw speed -----------------------------------------------
    // Safe because of step 1. This is noisy — encoder quantization
    // divided by a small dt jitters a lot — hence step 9.
    const float rawSpeed = deltaMeters / dt;

    // ---- 9. Low-pass filter -----------------------------------------
    // Exponential moving average. alpha_ in (0, 1]:
    //   1.0 = no filtering, speed_ jumps to rawSpeed
    //   0.1 = smooth, but lags
    // The PID sees this smoothed value, not rawSpeed, so it doesn't
    // chase sensor noise.
    speed_ += alpha_ * (rawSpeed - speed_);

    // ---- 10. Decide power -------------------------------------------
    float power;

    // Standstill special case: commanded target is zero AND the wheel
    // has actually stopped. If we let the PID keep running here, its
    // integral would slowly accumulate from encoder noise and later
    // cause a phantom nudge. Clear PID history and hold at 0.
    if (target_ == 0.0f && fabsf(speed_) < 0.01f) {
        pid_.reset();
        power = 0.0f;
    }
    else {
        // PID takes (setpoint_mps, measurement_mps, dt_s) and returns
        // a power in [-1, 1] to hand to the motor.
        power = pid_.compute(target_, speed_, dt);
    }

    // ---- 11. Drive the motor ----------------------------------------
    // setPower tells the motor WHAT to do. update advances its
    // internal slew-rate limiter by dt so it actually ramps there.
    // Skipping update(dt) freezes the ramp even though setPower
    // looks correct — the hardware never reaches the commanded value.
    motor_.setPower(power);
    motor_.update(dt);
}

// Goal: stop the wheel.
void WheelController::stop() {
    target_=0;
    pid_.reset();
    motor_.coast();
}

// Goal: stop fast.
void WheelController::brake() {
    target_=0;
    pid_.reset();
    motor_.brake();
}

void WheelController::reset(float value) {
    speed_    = value;
    target_   = value;
    firstRun_ = true;
    pid_.reset();
}
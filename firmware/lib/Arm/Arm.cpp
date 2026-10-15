// Arm.cpp -- see Arm.h
//
// Hardware: ESP32 LEDC, 50 Hz, 16 bit (Arduino-ESP32 core 2.x API:
// ledcSetup / ledcAttachPin / ledcWrite). Compiled without ARDUINO (env:native)
// the hardware calls are replaced by test hooks, so the logic runs on the PC.
#include "Arm.h"
#include "ServoConfig.h"     // ServoId (SERVO_LIFT, SERVO_REACH, SERVO_GRIPPER)

#include <cmath>
#include <stdint.h>

#ifdef ARDUINO
#include <Arduino.h>
#endif

using robot::ArmCmd;
using robot::ArmStatus;

static_assert(SERVO_COUNT <= Arm::MAX_SERVOS,
              "SERVO_COUNT in ServoConfig.h is larger than Arm::MAX_SERVOS");

// =====================================================================================
// Poses of the three sequences.
// PLACEHOLDERS: measure every angle on the real mechanism before the first run
// with a box. Degrees, 0..180. Test the first run with the arm unloaded.
// (Move these to config.h if the team keeps arm angles there.)
// =====================================================================================
namespace armpose {
constexpr float LIFT_UP        = 120.0f;   // PLACEHOLDER arm raised (travel / carrying)
constexpr float LIFT_DOWN      =  60.0f;   // PLACEHOLDER gripper at box height
constexpr float LIFT_DROP      =  70.0f;   // PLACEHOLDER height for releasing the box
constexpr float REACH_IN       =  30.0f;   // PLACEHOLDER retracted
constexpr float REACH_OUT      = 120.0f;   // PLACEHOLDER extended toward the box
constexpr float GRIPPER_OPEN   = 120.0f;   // PLACEHOLDER wider than the 6 cm box
constexpr float GRIPPER_CLOSED =  40.0f;   // PLACEHOLDER holding the box
}  // namespace armpose

// =====================================================================================
// Constants
// =====================================================================================
namespace {
constexpr uint32_t PWM_FREQ_HZ    = 50;
constexpr uint8_t  PWM_BITS       = 16;
constexpr float    PWM_PERIOD_US  = 20000.0f;          // 1 / 50 Hz
constexpr float    PWM_MAX_DUTY   = 65535.0f;          // 2^16 - 1
constexpr uint32_t BEGIN_PAUSE_MS = 120;               // between servos in begin()
constexpr float    MAX_DT_S       = 0.1f;              // a stalled task must not cause a jump
constexpr float    AT_STOW        = -1.0f;             // step target: "this servo's stowDeg"
}  // namespace

// =====================================================================================
// Hardware layer
// =====================================================================================
#ifdef ARDUINO

static void hwAttach(uint8_t pin, uint8_t channel) {
    ledcSetup(channel, PWM_FREQ_HZ, PWM_BITS);
    ledcAttachPin(pin, channel);
}
static void hwWrite(uint8_t channel, uint32_t duty) { ledcWrite(channel, duty); }
static void hwDelayMs(uint32_t ms) { delay(ms); }

#else   // host build: record what would have been written (read by test_arm)

uint32_t arm_test_duty[16];
uint32_t arm_test_delayMs     = 0;
int      arm_test_attachCalls = 0;

static void hwAttach(uint8_t, uint8_t) { arm_test_attachCalls++; }
static void hwWrite(uint8_t channel, uint32_t duty) { arm_test_duty[channel & 15] = duty; }
static void hwDelayMs(uint32_t ms) { arm_test_delayMs += ms; }

#endif

static float clampDeg(float d) {
    if (d < 0.0f) return 0.0f;
    if (d > 180.0f) return 180.0f;
    return d;
}

// =====================================================================================
// Configuration
// =====================================================================================

void Arm::begin(const ServoConfig* cfg, uint8_t count) {
    // Any failure leaves count_ = 0, so start() refuses to run.
    count_ = 0;
    seq_ = nullptr;
    seqLen_ = 0;
    stepIdx_ = 0;
    arrived_ = false;
    settleLeftS_ = 0.0f;
    status_ = ArmStatus::IDLE;

    if (cfg == nullptr || count < 1 || count > MAX_SERVOS) return;
    for (uint8_t i = 0; i < count; i++) {
        if (cfg[i].maxUs <= cfg[i].minUs) return;                       // bad pulse range
        if (!std::isfinite(cfg[i].maxSpeedDps) || cfg[i].maxSpeedDps <= 0.0f) return;
        if (!std::isfinite(cfg[i].stowDeg)) return;
    }

    for (uint8_t i = 0; i < count; i++) {
        servos_[i].cfg = cfg[i];
        servos_[i].currentDeg = clampDeg(cfg[i].stowDeg);
        servos_[i].targetDeg = servos_[i].currentDeg;
    }
    count_ = count;

    // One servo at a time, with a pause, so the startup current stays low.
    // There is no feedback: each servo jumps to stowDeg from wherever it is.
    for (uint8_t i = 0; i < count_; i++) {
        hwAttach(servos_[i].cfg.pin, servos_[i].cfg.channel);
        writeAngle(i, servos_[i].currentDeg);
        hwDelayMs(BEGIN_PAUSE_MS);
    }
}

void Arm::setHoldCheck(bool (*isHolding)()) { isHolding_ = isHolding; }

// =====================================================================================
// Runtime
// =====================================================================================

bool Arm::start(ArmCmd cmd) {
    if (count_ == 0 || status_ == ArmStatus::BUSY) return false;

    // Each step: servo, target angle, settle time after arrival (s), check hold sensor.
    // Exactly ONE servo per step.
    static const Step kPick[] = {
        { SERVO_GRIPPER, armpose::GRIPPER_OPEN,   0.3f, false },
        { SERVO_REACH,   armpose::REACH_OUT,      0.3f, false },
        { SERVO_LIFT,    armpose::LIFT_DOWN,      0.3f, false },
        { SERVO_GRIPPER, armpose::GRIPPER_CLOSED, 0.5f, false },
        { SERVO_LIFT,    armpose::LIFT_UP,        0.3f, true  },   // box must be held now
        { SERVO_REACH,   armpose::REACH_IN,       0.3f, true  },
    };
    static const Step kDrop[] = {
        { SERVO_LIFT,    armpose::LIFT_DROP,      0.3f, false },
        { SERVO_REACH,   armpose::REACH_OUT,      0.3f, false },
        { SERVO_GRIPPER, armpose::GRIPPER_OPEN,   0.5f, false },
        { SERVO_REACH,   armpose::REACH_IN,       0.3f, false },
        { SERVO_LIFT,    armpose::LIFT_UP,        0.3f, false },
    };
    static const Step kStow[] = {
        { SERVO_LIFT,    AT_STOW,                 0.3f, false },
        { SERVO_REACH,   AT_STOW,                 0.3f, false },
        { SERVO_GRIPPER, AT_STOW,                 0.3f, false },
    };

    const Step* seq = nullptr;
    uint8_t len = 0;
    switch (cmd) {
        case ArmCmd::PICK: seq = kPick; len = (uint8_t)(sizeof(kPick) / sizeof(kPick[0])); break;
        case ArmCmd::DROP: seq = kDrop; len = (uint8_t)(sizeof(kDrop) / sizeof(kDrop[0])); break;
        case ArmCmd::STOW: seq = kStow; len = (uint8_t)(sizeof(kStow) / sizeof(kStow[0])); break;
        default: return false;
    }

    // Refuse a table that cannot run on this arm (nothing has moved yet).
    for (uint8_t i = 0; i < len; i++) {
        const Step& s = seq[i];
        if (s.servo >= count_) return false;
        const bool angleOk = (s.targetDeg == AT_STOW) ||
                             (std::isfinite(s.targetDeg) && s.targetDeg >= 0.0f &&
                              s.targetDeg <= 180.0f);
        if (!angleOk) return false;
        if (!std::isfinite(s.settleS) || s.settleS < 0.0f) return false;
    }

    status_ = ArmStatus::BUSY;          // also clears a latched DONE / FAILED
    beginSequence(seq, len);
    return true;
}

void Arm::update(float dt) {
    if (status_ != ArmStatus::BUSY || seq_ == nullptr) return;
    if (!std::isfinite(dt) || dt <= 0.0f) return;
    if (dt > MAX_DT_S) dt = MAX_DT_S;

    const Step& step = seq_[stepIdx_];
    ServoState& s = servos_[step.servo];

    if (!arrived_) {
        const float maxMove = s.cfg.maxSpeedDps * dt;
        const float diff = s.targetDeg - s.currentDeg;
        if (std::fabs(diff) <= maxMove) {
            s.currentDeg = s.targetDeg;
            arrived_ = true;
        } else {
            s.currentDeg += (diff > 0.0f) ? maxMove : -maxMove;
        }
        writeAngle(step.servo, s.currentDeg);
    } else {
        settleLeftS_ -= dt;
    }

    if (arrived_ && settleLeftS_ <= 0.0f) {
        if (step.checkHold && isHolding_ != nullptr && !isHolding_()) {
            finish(ArmStatus::FAILED);          // servos keep their angle
            return;
        }
        stepIdx_++;
        if (stepIdx_ >= seqLen_) finish(ArmStatus::DONE);
        else beginStep();
    }
}

void Arm::abort() {
    if (status_ != ArmStatus::BUSY) return;     // nothing running: keep DONE / IDLE
    if (seq_ != nullptr) {
        ServoState& s = servos_[seq_[stepIdx_].servo];
        s.targetDeg = s.currentDeg;             // hold here; PWM keeps the last duty
    }
    finish(ArmStatus::FAILED);
}

void Arm::acknowledge() {
    if (status_ == ArmStatus::DONE || status_ == ArmStatus::FAILED)
        status_ = ArmStatus::IDLE;
}

float Arm::angle(uint8_t servo) const {
    if (servo >= count_) return NAN;
    return servos_[servo].currentDeg;
}

// =====================================================================================
// Private
// =====================================================================================

void Arm::writeAngle(uint8_t servo, float deg) {
    if (servo >= MAX_SERVOS) return;
    const ServoConfig& c = servos_[servo].cfg;
    const float d = clampDeg(deg);
    const float us = (float)c.minUs + (float)(c.maxUs - c.minUs) * (d / 180.0f);
    hwWrite(c.channel, (uint32_t)(us / PWM_PERIOD_US * PWM_MAX_DUTY + 0.5f));
}

void Arm::beginSequence(const Step* seq, uint8_t len) {
    seq_ = seq;
    seqLen_ = len;
    stepIdx_ = 0;
    beginStep();
}

void Arm::beginStep() {
    const Step& step = seq_[stepIdx_];
    ServoState& s = servos_[step.servo];
    s.targetDeg = clampDeg(step.targetDeg == AT_STOW ? s.cfg.stowDeg : step.targetDeg);
    arrived_ = false;
    settleLeftS_ = step.settleS;
}

void Arm::finish(ArmStatus result) {
    status_ = result;
    seq_ = nullptr;
    seqLen_ = 0;
    stepIdx_ = 0;
    arrived_ = false;
}

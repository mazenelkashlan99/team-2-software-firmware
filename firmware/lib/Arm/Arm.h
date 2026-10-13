#pragma once
#include <stdint.h>
#include "robot_interfaces.h"   // robot::ArmCmd (PICK, DROP, STOW), robot::ArmStatus (IDLE, BUSY, DONE, FAILED)

// Servo arm: executes pick / drop / stow as non-blocking sequences.
//
// Rules the design enforces:
//   - ONE servo moves at a time. Each step of a sequence names exactly one
//     servo, and the next step starts only after the current servo has
//     arrived and settled. This keeps the peak current on the single buck low.
//   - Servos move in small steps (limited speed, deg/s), never in one jump.
//   - Hobby servos have no feedback: "arrived" means "the commanded angle has
//     reached the target and the settle time has passed", not a measurement.
//
// No Arduino.h here. The LEDC calls (50 Hz PWM) live in Arm.cpp.
// Units: degrees, degrees per second, microseconds (pulse width), seconds.
//
// Sequences are tables of steps defined in Arm.cpp, using the angles from
// config.h. Measure the real angles on the real mechanism.

class Arm {
public:
    static constexpr uint8_t MAX_SERVOS = 3;      // e.g. lift, reach, gripper

    struct ServoConfig {
        uint8_t  pin;           // signal pin (not a strapping pin, not GPIO 6-11)
        uint8_t  channel;       // LEDC channel, on its own pair, away from the motors
        uint16_t minUs;         // pulse width at 0 degrees   (measure it, ~500-1000)
        uint16_t maxUs;         // pulse width at 180 degrees (measure it, ~2000-2500)
        float    stowDeg;       // safe angle used at startup and by stow()
        float    maxSpeedDps;   // speed limit, degrees per second
    };

    Arm() = default;

    // ---- Configuration -----------------------------------------------

    // Set up PWM and move every servo to its stowDeg, ONE AT A TIME, with a short
    // pause between them. Blocks for a few hundred ms. Call once in setup().
    // count must be 1..MAX_SERVOS.
    void begin(const ServoConfig* cfg, uint8_t count);

    // Optional: a function returning true when a box is actually held (e.g. from
    // the IR sensor). Steps marked checkHold use it, and the sequence ends in
    // FAILED if it returns false. A function pointer keeps Arm independent of Ir.
    void setHoldCheck(bool (*isHolding)());

    // ---- Runtime -----------------------------------------------------

    // Start a sequence. Returns false (and changes nothing) if a sequence is
    // already running or begin() has not been called.
    bool start(robot::ArmCmd cmd);

    // Advance the active sequence by dt seconds. Call at a fixed rate (about 50 Hz).
    void update(float dt);

    // Stop now: the sequence ends, servos HOLD their current angle (so a held box
    // is not dropped), the status becomes FAILED.
    void abort();

    // Reset DONE or FAILED to IDLE after the result has been read.
    void acknowledge();

    // ---- Readings ----------------------------------------------------

    robot::ArmStatus status() const { return status_; }   // latched until acknowledge() or start()
    bool  busy() const { return status_ == robot::ArmStatus::BUSY; }
    float angle(uint8_t servo) const;                     // current commanded angle, degrees

private:
    struct ServoState {
        ServoConfig cfg;
        float       currentDeg = 0.0f;   // what is being commanded now
        float       targetDeg  = 0.0f;   // where the active step wants it
    };

    // One step of a sequence: move ONE servo to an angle, wait for it to settle.
    struct Step {
        uint8_t servo;        // index into servos_
        float   targetDeg;
        float   settleS;      // pause after arrival, before the next step
        bool    checkHold;    // verify the hold sensor after this step
    };

    void writeAngle(uint8_t servo, float deg);    // angle -> pulse width -> LEDC duty
    void beginSequence(const Step* seq, uint8_t len);
    void beginStep();                             // load the target of stepIdx_
    void finish(robot::ArmStatus result);

    ServoState servos_[MAX_SERVOS];
    uint8_t    count_ = 0;

    robot::ArmStatus status_ = robot::ArmStatus::IDLE;

    const Step* seq_      = nullptr;   // the running sequence (a table in Arm.cpp)
    uint8_t     seqLen_   = 0;
    uint8_t     stepIdx_  = 0;
    bool        arrived_  = false;     // active servo reached its target
    float       settleLeftS_ = 0.0f;   // settle time still to wait

    bool (*isHolding_)() = nullptr;
};
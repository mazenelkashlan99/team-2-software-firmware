#pragma once
#include <stdint.h>

// Servo hardware description for the arm. Pure data: no Arduino.h, no logic.
// Every value marked PLACEHOLDER must be replaced with a measured one.
//
// Units: degrees, degrees per second, microseconds.
//
// LEDC rule: channels share timers in pairs (0-1, 2-3, 4-5, 6-7). All servos use
// 50 Hz, so they may share pairs with each other, but NOT with the motors (20 kHz).
// Motors use channels 0-3 (two driven wheels), so servos start at 4.
// With four driven wheels the motors take 0-7 and servos start at 8.

struct ServoConfig {
    uint8_t  pin;           // signal pin: not a strapping pin (0, 2, 5, 12, 15),
                            // not GPIO 6-11, and an OUTPUT-capable pin (not 34-39)
    uint8_t  channel;       // LEDC channel
    uint16_t minUs;         // pulse width at 0 degrees    (measure it; datasheet is approximate)
    uint16_t maxUs;         // pulse width at 180 degrees  (measure it)
    float    stowDeg;       // safe angle used at startup and by stow()
    float    maxSpeedDps;   // speed limit, degrees per second
};

// ---- How many servos, and which index is which -------------------------------------
// TODO: set from the mechanical design. Change SERVO_COUNT and the table together.
constexpr uint8_t SERVO_COUNT = 3;       // PLACEHOLDER

enum ServoId : uint8_t {
    SERVO_LIFT    = 0,   // raises and lowers the arm
    SERVO_REACH   = 1,   // extends toward the box
    SERVO_GRIPPER = 2,   // opens and closes on the 6 cm box
};

// ---- Per-servo hardware values ------------------------------------------------------
// TODO: pins from the PCB, pulse range from testing each servo,
#pragma once
#include <Arduino.h>

// Analog IR reflective sensor with filtering and a detected / not detected output.
//
// Reads one analog pin. It knows nothing about the arm, the autonomy or the
// pose: it only answers "is something there?". The caller decides what that means
// (box in the gripper, box row ahead, boundary line under the robot).
//
// Use ADC1 pins only (GPIO 32 to 39). ADC2 does not work while WiFi is on.
// The sensor output must never exceed 3.3 V at the pin.
//
// Raw values are ADC counts, 0 to 4095 (12 bits). Thresholds are in the same
// counts and must be calibrated on the real floor and boxes: IR readings change
// with surface color and room light.

class Ir {
public:
    explicit Ir(uint8_t pin);

    // ---- Configuration -----------------------------------------------

    // Set up the pin. Call once in setup().
    void begin();

    // Low-pass filter on the readings. alpha in (0, 1]: 1 = no filtering.
    void setFilter(float alpha);

    // Two thresholds, in ADC counts, with low < high (hysteresis).
    // detected() turns true when the filtered value rises above `high`,
    // and false again only when it falls below `low`. Without the gap,
    // a reading sitting near one threshold would flicker.
    void setThresholds(uint16_t low, uint16_t high);

    // Some modules read HIGH when nothing is there (output drops as the object
    // gets closer). Set true to flip the reading so "more = closer" always holds.
    void setInverted(bool inverted);

    // ---- Runtime -----------------------------------------------------

    // Take one sample, filter it, and update detected(). Call at a fixed rate
    // (for example 100 Hz, or slower for a gripper check).
    void update();

    // ---- Readings ----------------------------------------------------

    uint16_t raw()      const { return raw_; }        // last raw sample, after inversion
    float    filtered() const { return filtered_; }   // low-pass filtered, 0 to 4095
    bool     detected() const { return detected_; }   // with hysteresis

private:
    uint8_t  pin_;
    bool     inverted_ = false;

    float    alpha_    = 0.3f;
    uint16_t low_      = 1500;    // placeholders: calibrate on the real robot
    uint16_t high_     = 2500;

    uint16_t raw_      = 0;
    float    filtered_ = 0.0f;
    bool     detected_ = false;
    bool     firstRun_ = true;    // the first sample seeds the filter, no ramp from 0
};
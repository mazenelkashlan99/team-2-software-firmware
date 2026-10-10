#include "Ir.h"


// Constructor only stores the pin.
//
// Why no analogRead() / pinMode() here: this object may be constructed
// as a global, before setup() runs and before the Arduino core has
// initialized the ADC. Touching hardware before init is undefined
// behavior — the pin mode register may not exist yet, or the ADC may
// not be enabled. All hardware setup goes in begin(), which the sketch
// calls from setup() once the core is ready.
Ir::Ir(uint8_t pin) : pin_(pin) {}


// Prepare the pin for analog reading. Call once from setup().
void Ir::begin() {
    // No pinMode() needed for an analog input on the ESP32. analogRead()
    // routes the pin to the ADC through the GPIO matrix, and the pin's
    // digital input/output buffers are bypassed. Setting INPUT_PULLUP
    // here would actually be harmful: the pull-up fights the sensor's
    // analog output and skews the reading.

    // The ESP32 ADC defaults to 0..1.1 V at the pin. Our sensor swings
    // up to ~3.3 V, so set 11 dB attenuation, which gives roughly the
    // full 0..3.3 V range at 12-bit resolution (0..4095 counts).
    analogSetPinAttenuation(pin_, ADC_11db);

    // Reset filter state so the first update() seeds from the real
    // sensor reading instead of drifting up from 0.
    firstRun_ = true;
    filtered_ = 0.0f;
    detected_ = false;
    raw_      = 0;
}


// Valid range for alpha is (0, 1]:
//   1.0 = no filtering (snap to the latest sample)
//   near 0 = heavy filtering (smooth but slow)
// 0 would freeze filtered_ forever; >1 would overshoot and oscillate.
void Ir::setFilter(float alpha) {
    if (alpha > 0.0f && alpha <= 1.0f) {
        alpha_ = alpha;
    }
}


// Two thresholds with low < high (hysteresis).
// - Swap if the caller passed them reversed, same as PID::setIntegralLimits.
// - Clamp to the ADC's 12-bit range: values above 4095 or below 0 can
//   never be reached, which would leave the sensor stuck in one state.
void Ir::setThresholds(uint16_t low, uint16_t high) {
    if (low > high) {
        uint16_t tmp = low;
        low  = high;
        high = tmp;
    }

    // 4095 is the max for a 12-bit ADC; nothing above that is reachable.
    if (low  > 4095) low  = 4095;
    if (high > 4095) high = 4095;

    low_  = low;
    high_ = high;
}


void Ir::setInverted(bool inverted) {
    inverted_ = inverted;
}


// One sample, one filter step, one hysteresis decision.
void Ir::update() {
    // 1. Read the pin.
    // analogRead returns 0..4095 on the ESP32. When inverted_, flip it
    // so "higher = closer" always holds, regardless of which way the
    // module's output swings. 4095 is the ADC's full scale.
    uint16_t sample = static_cast<uint16_t>(analogRead(pin_));
    if (inverted_) {
        sample = 4095 - sample;
    }
    raw_ = sample;

    // 2. Low-pass filter.
    // On the very first call, seed filtered_ with the raw value. Starting
    // from 0 would make the filter ramp up over several calls, which
    // would (a) look like a false "not detected" at boot and (b) delay
    // the first real reading. Same formula as WheelController's speed
    // filter: move a fraction alpha_ of the way toward the new sample.
    if (firstRun_) {
        filtered_ = static_cast<float>(raw_);
        firstRun_ = false;
    } else {
        filtered_ += alpha_ * (static_cast<float>(raw_) - filtered_);
    }

    // 3. Hysteresis.
    //   - Above high_  : detected = true.
    //   - Below low_   : detected = false.
    //   - In between   : keep the previous state. That's the whole point
    //                    of hysteresis: a reading wobbling around one
    //                    threshold won't flicker the output.
    if (filtered_ > static_cast<float>(high_)) {
        detected_ = true;
    } else if (filtered_ < static_cast<float>(low_)) {
        detected_ = false;
    }
    // else: no change
}
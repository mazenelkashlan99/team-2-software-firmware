#pragma once

#include <stdint.h>

// Incremental quadrature encoder read via the ESP32 PCNT peripheral.
//
// One instance = one wheel. The PCNT unit counts in hardware, so no ISR
// is needed and no edges are missed. update() must be called at a fixed
// rate; it reads the hardware counter, accumulates into a 32-bit total,
// and produces a low-pass-filtered speed.
//
// Wiring:
//   - A and B are the two encoder channels (quadrature).
//   - Encoder VCC to 3.3 V, GND common with the ESP32.
//   - Do NOT use strapping pins (0, 2, 5, 12, 15) or
//     input-only pins (34-39) for A or B unless you have external pull-ups.
//
// PCNT units on a classic ESP32: 0-7 (two motors need two units).
// On ESP32-S3: 0-3. On ESP32-C3: 0-1.

class Encoder {
public:
    // pinA, pinB : quadrature channels
    // pcntUnit   : PCNT unit index, unique per encoder (0..N-1)
    Encoder(uint8_t pinA, uint8_t pinB, uint8_t pcntUnit);

    // Configure the PCNT unit and start counting. Safe to call once.
    void begin();

    // Call at a fixed rate. dt is seconds since the previous update().
    // Reads the hardware counter, updates ticks_, delta_, and speed_.
    void update(float dt);

    // ---- Position ---------------------------------------------------

    // Cumulative signed ticks since reset(). int32_t is safe for a very
    // long run at any realistic wheel speed.
    int32_t getTicks() const;

    // Signed ticks accumulated during the last update() call.
    // Resets on each update(). Use this for odometry integration.
    int32_t getDeltaTicks() const;

    // Cumulative signed distance in meters, using ticksPerMeter_.
    // Invalid until setTicksPerMeter() is called with the real value.
    float getDistanceM() const;

    // ---- Speed ------------------------------------------------------

    // Low-pass filtered speed in ticks per second.
    float getTicksPerSec() const;

    // Low-pass filtered speed in meters per second.
    // Invalid until setTicksPerMeter() is called.
    float getSpeedMps() const;

    // ---- State ------------------------------------------------------

    // Pause the PCNT unit, zero the counters and the filter, resume.
    // Safe to call while the wheel is spinning.
    void reset();

    // ---- Configuration ---------------------------------------------

    // Encoder ticks per meter of wheel travel. Usually computed in
    // config.h from PPR, gear ratio, and wheel diameter:
    //   ticksPerMeter = (PPR * 4 * gearRatio) / (pi * wheelDiameter_m)
    // The *4 accounts for quadrature decoding of both edges of both
    // channels.
    void setTicksPerMeter(float ticksPerMeter);

    // Low-pass filter coefficient in (0, 1]. Larger = faster response,
    // noisier. Smaller = smoother, laggier. Typical starting value 0.3.
    void setFilterAlpha(float alpha);

private:
    uint8_t pinA_;
    uint8_t pinB_;
    uint8_t unit_;               // PCNT unit index, unique per encoder instance
    int16_t lastRaw_   = 0;      // previous raw PCNT reading (16-bit wrap)
    int32_t ticks_     = 0;      // cumulative signed ticks (32-bit)
    int32_t delta_     = 0;      // ticks since the last update()

    float   speedTps_  = 0.0f;   // filtered speed, ticks per second
    float   ticksPerMeter_ = 0.0f;
    float   alpha_         = 0.3f;
};
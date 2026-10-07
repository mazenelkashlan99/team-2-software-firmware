#include "Encoder.h"
#include "driver/pcnt.h"

Encoder::Encoder(uint8_t pinA, uint8_t pinB, uint8_t pcntUnit){

    pinA_=pinA;
    pinB_=pinB;
    unit_=pcntUnit;

}

void Encoder::begin() {
    lastRaw_ = 0;
    // ---- Channel 0: count edges on A, B decides direction ----
    pcnt_config_t cfgA = {};
    cfgA.pulse_gpio_num = pinA_;
    cfgA.ctrl_gpio_num  = pinB_;
    cfgA.channel        = PCNT_CHANNEL_0;
    cfgA.unit           = (pcnt_unit_t)unit_;
    cfgA.pos_mode       = PCNT_COUNT_INC;      // rising edge  → +1
    cfgA.neg_mode       = PCNT_COUNT_DEC;      // falling edge → -1
    cfgA.lctrl_mode     = PCNT_MODE_REVERSE;   // B low  → reverse
    cfgA.hctrl_mode     = PCNT_MODE_KEEP;      // B high → keep
    cfgA.counter_h_lim  = 32767;
    cfgA.counter_l_lim  = -32768;
    pcnt_unit_config(&cfgA);

    // ---- Channel 1: count edges on B, A decides direction ----
    pcnt_config_t cfgB = {};
    cfgB.pulse_gpio_num = pinB_;
    cfgB.ctrl_gpio_num  = pinA_;
    cfgB.channel        = PCNT_CHANNEL_1;
    cfgB.unit           = (pcnt_unit_t)unit_;
    cfgB.pos_mode       = PCNT_COUNT_INC;
    cfgB.neg_mode       = PCNT_COUNT_DEC;
    cfgB.lctrl_mode     = PCNT_MODE_KEEP;      // A low  → keep
    cfgB.hctrl_mode     = PCNT_MODE_REVERSE;   // A high → reverse
    cfgB.counter_h_lim  = 32767;
    cfgB.counter_l_lim  = -32768;
    pcnt_unit_config(&cfgB);

    // ---- Glitch filter: ignore pulses shorter than ~12.5 µs ----
    pcnt_set_filter_value((pcnt_unit_t)unit_, 1000);
    pcnt_filter_enable((pcnt_unit_t)unit_);

    // ---- Start counting from 0 ----
    pcnt_counter_pause((pcnt_unit_t)unit_);
    pcnt_counter_clear((pcnt_unit_t)unit_);
    pcnt_counter_resume((pcnt_unit_t)unit_);
}

void Encoder::update(float dt){
    // dt : seconds since the last time update() was called

    // Read raw hardware counter (16-bit, wraps around)
    // writes the current hardware count into raw
    // raw = means where the wheel is right now, according to the hardware.
    int16_t raw = 0;
    pcnt_get_counter_value((pcnt_unit_t)unit_, &raw);
    // Compute delta since last call, handling 16-bit wraparound.
    int16_t diff = raw - lastRaw_;
    // lastRaw_ is what raw was on the previous call to update().
    lastRaw_ = raw;
    delta_ = diff;
    ticks_ += diff; // accumulated ticks since reset()

    float rawTps = (dt > 0.0f) ? (diff / dt) : 0.0f;

    // 5. Low-pass filter (exponential moving average)
    // smoothing factor for the low-pass filter
    speedTps_ = alpha_ * rawTps + (1.0f - alpha_) * speedTps_;
}

int32_t Encoder::getTicks() const{
    return ticks_;
}

int32_t Encoder::getDeltaTicks() const{
    return delta_;
}

float Encoder::getDistanceM() const {
    return (ticksPerMeter_ > 0.0f) ? ticks_ / ticksPerMeter_ : 0.0f;
}

float Encoder::getSpeedMps() const {
    return (ticksPerMeter_ > 0.0f) ? speedTps_ / ticksPerMeter_ : 0.0f;
}


float Encoder::getTicksPerSec() const {
    return speedTps_;
}

void Encoder::reset() {
    pcnt_counter_pause((pcnt_unit_t)unit_);
    pcnt_counter_clear((pcnt_unit_t)unit_);
    lastRaw_  = 0;
    ticks_    = 0;
    delta_    = 0;
    speedTps_ = 0.0f;
    pcnt_counter_resume((pcnt_unit_t)unit_);
}

void Encoder::setTicksPerMeter(float ticksPerMeter){
    if (ticksPerMeter > 0.0f) {
        ticksPerMeter_ = ticksPerMeter;
    }
}

void Encoder::setFilterAlpha(float alpha){
    if (alpha > 0.0f && alpha <= 1.0f) {
        alpha_ = alpha;
    }
}
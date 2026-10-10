#include "Motor-Controller.h"

// Goal: remember where this motor is wired.
MotorController::MotorController(uint8_t pin1, uint8_t pin2, uint8_t LEDC_channel_one, uint8_t LEDC_channel_two){
    pin1_ = pin1;
    pin2_ = pin2;
    ch1_ = LEDC_channel_one;
    ch2_ = LEDC_channel_two;
}

// Goal: get the PWM hardware ready for both pins.
void MotorController::begin(uint32_t pwm_freqHz, uint8_t bits){
    // compute maxDuty_ from bits  (2^bits - 1)
    maxDuty_= (1 << bits) - 1; // same as (2^bits - 1)
    // configure both channels   -> ledcSetup(channel, freq, bits)
    ledcSetup(ch1_, pwm_freqHz, bits);
    ledcSetup(ch2_, pwm_freqHz, bits);
    // attach each pin to its channel -> ledcAttachPin(pin, channel)
    ledcAttachPin(pin1_, ch1_);
    ledcAttachPin(pin2_, ch2_);
    // start in a safe state     -> coast()
    coast();
}

// Goal: recoord power correctly
void MotorController::setPower(float p){
    if (p > 1){
        p = 1;
    }
    if (p < -1){
        p=-1;
    }
    // after clamping the p
    target_ = p;
    if (slew_ <= 0) { // Slew rate is the rate of change of the power
        current_ = target_;
        apply(current_);
    }
}

// Goal: move the motor's real power toward what was asked, at a controlled pace.
void MotorController::update(float dt){
    // dt is the time in seconds since the last update() call
    // slew limit off: setPower() already applied the power, nothing to ramp
    if (slew_ <= 0){
        return;
    }

    // the most the power may change during this tick (power per second * seconds)
    float maxStep = slew_ * dt;

    // how far the applied power still is from the target (signed: + means go up, - means go down)
    float diff = target_ - current_;

    // clamp the gap to one step; if it is already smaller than a step, it is left unchanged,
    // so current_ lands exactly on target_ with no overshoot
    if (diff > maxStep)  diff = maxStep;
    if (diff < -maxStep) diff = -maxStep;

    // move the applied power by the limited step
    current_ += diff;

    // send the new power to the pins
    apply(current_);
}

// Goal: turn a signed power into signals on the two pins.
void MotorController::apply(float p){

    if (p == 0) {
        ledcWrite(ch1_, 0);
        ledcWrite(ch2_, 0);
        return;
    }    
    
    float mag = fabsf(p); // abs value of p

    //shifting everything up so the smallest request instead of 0%, and the biggest request is still 100%:
    mag = deadband_ + mag * (1 - deadband_); // low + value × (high − low).
    
    // 0.5F was added because turns truncation into rounding-to-nearest Because C++ converts float to uint32_t
    //  by truncating toward zero, not by rounding. Adding 0.5f before the cast turns truncation into rounding-to-nearest.
    uint32_t duty = (uint32_t)(mag * maxDuty_ + 0.5f);

    if (p > 0) { // write to one , set the other to 0
        ledcWrite(ch1_, duty);   
        ledcWrite(ch2_, 0);     
    } 
    else {
        ledcWrite(ch1_, 0);    // reverse: the opposite
        ledcWrite(ch2_, duty);
    }

}

// Goal: stop fast.
void MotorController::brake(){
    target_=0;
    current_=0;
    ledcWrite(ch1_, maxDuty_);
    ledcWrite(ch2_, maxDuty_);
}

// Goal: stop gently.
void MotorController::coast(){
    target_=0;
    current_=0;
    ledcWrite(ch1_, 0);
    ledcWrite(ch2_, 0);
}

// Setter : Deadband
void MotorController::setDeadband(float db){
    if (db < 0.0f){
        db = 0.0f;
    }
    if (db > 1.0f){
        db = 0.9999f;
    } 
    deadband_ = db;
}

// Setter : SlewRate
void MotorController::setSlewRate(float perSec) {
    if (perSec>=0){
        slew_=perSec;
    }
}

// Getter : Power
float MotorController::getPower() const{ 
    return current_; 
}
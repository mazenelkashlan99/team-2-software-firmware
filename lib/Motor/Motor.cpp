#include "Motor.h"

// ---------- Constructor ----------
// Store the pins and channels in the private members.
// Use the initializer list: Motor::Motor(...) : pin1_(pin1), ... { }
Motor::Motor(int pin1, int pin2, int LEDC_channel_one, int LEDC_channel_two)
{
    // TODO: nothing else is needed here; no hardware calls yet (that's begin's job)
}

// ---------- begin ----------
void Motor::begin(int pwm_freqHz, int bits)
{
    // TODO 1: compute maxDuty_ from bits  (2^bits - 1)
    maxDuty_ = 2^bits - 1;
    // TODO 2: configure both channels   -> ledcSetup(channel, freq, bits)
    // TODO 3: attach each pin to its channel -> ledcAttachPin(pin, channel)
    // TODO 4: start in a safe state     -> coast()
}

// ---------- setPower ----------
void Motor::setPower(float p)
{
    // TODO 1: clamp p to the range -1.0 .. +1.0 (constrain)
    // TODO 2: store it in target_
    // TODO 3: if slew limiting is off (slew_ <= 0), also set current_ = target_
    //         and call apply(current_) right away
}

// ---------- update ----------
void Motor::update(float dt)
{
    // TODO 1: if slew_ <= 0, there is nothing to do -> return
    // TODO 2: maxStep = slew_ * dt   (the biggest change allowed this tick)
    // TODO 3: diff = target_ - current_
    // TODO 4: limit diff to the range -maxStep .. +maxStep
    // TODO 5: current_ += diff
    // TODO 6: apply(current_)
}

// ---------- apply (private) ----------
void Motor::apply(float p)
{
    // TODO 1: if p == 0 -> coast and return
    // TODO 2: mag = fabsf(p)   (the size, without the sign)
    // TODO 3: deadband compensation: map mag from (0..1] to [deadband_..1]
    //         mag = deadband_ + mag * (1 - deadband_)
    // TODO 4: duty = (int)(mag * maxDuty_)
    // TODO 5: if p > 0 -> PWM on channel one, 0 on channel two   (ledcWrite)
    //         else     -> 0 on channel one, PWM on channel two
}

// ---------- brake ----------
void Motor::brake()
{
    // TODO 1: target_ and current_ = 0
    // TODO 2: both channels at maxDuty_  (both inputs high)
}

// ---------- coast ----------
void Motor::coast()
{
    // TODO 1: target_ and current_ = 0
    // TODO 2: both channels at 0  (both inputs low)
}

// ---------- setters and getter ----------
void Motor::setDeadband(float db)     { /* TODO: store in deadband_ */ }
void Motor::setSlewRate(float perSec) { /* TODO: store in slew_ */ }
float Motor::getPower() const         { /* TODO: return current_ */ return 0; }
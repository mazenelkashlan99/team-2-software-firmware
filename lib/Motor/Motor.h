#pragma once
#include <Arduino.h>

class Motor {
  public:
    // constructor
    Motor(uint8_t pin1, uint8_t pin2, uint8_t LEDC_channel_one, uint8_t LEDC_channel_two);

    // pwm_freqHz: how fast the PWM switches
    // bits: PWM resolution; the duty value goes from 0 to (2^bits - 1)
    void begin(uint32_t pwm_freqHz, uint8_t bits);

    // power range from -1 to +1 and sign indicates direction
    void setPower(float p);

    // dt : time since last call
    void update(float dt);

    void brake();   // both inputs high: stops quickly

    void coast();   // both inputs low: free spin (stop slowly)

    // smallest power that actually makes the motor move
    void setDeadband(float db);

    // maximum change in power per second (limits how fast the power is allowed to change)
    void setSlewRate(float perSec);

    // getter for power
    float getPower() const;

  private:

    void apply(float p);           // converts power to PWM duty and writes the pins

    uint8_t pin1_, pin2_;
    uint8_t ch1_, ch2_;
    uint32_t maxDuty_ = 0;
    float target_  = 0; // clamped power
    float current_ = 0; //
    float deadband_ = 0.0f;
    float slew_ = 0.0f; // maximum power change per second
};
#pragma once
#include <Arduino.h>

class Motor {    
  public:
    // constructor
    Motor(int pin1, int pin2, int LEDC_channel_one, int LEDC_channel_two);

    // pwm_freqHz: how fast the PWM switches
    // bits : PWM resolution
    // decides the largest duty value (maxDuty_ = 2^bits - 1),
    void begin(int pwm_freqHz, int bits);

    // power range from -1  to +1 and sign indicates direction
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

    int pin1_, pin2_;
    int ch1_, ch2_;
    int maxDuty_ = 0;
    float target_  = 0;
    float current_ = 0;
    float deadband_ = 0.0f;
    float slew_ = 0.0f;

};


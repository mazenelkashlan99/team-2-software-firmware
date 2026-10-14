#pragma once

class RateLimiter {
public:
    RateLimiter() = default;

    void  setMaxRate(float unitsPerSec);
    float update(float target, float dt);
    void  reset(float value);

    float value() const { return value_; }

private:
    float maxRate_ = 0.0f;
    float value_   = 0.0f;
};
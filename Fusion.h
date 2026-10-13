#pragma once

class Fusion {
public:
  explicit Fusion(float alpha = 0.98f, float slipRateRadS = 1.5f)
      : alpha_(alpha), slipRate_(slipRateRadS) {}

  void reset(float theta = 0.0f);

  float update(float dThetaEnc, float gz, float dt, bool imuValid);

  float heading() const { return theta_; }
  bool slipDetected() const { return slip_; }

private:
  float alpha_, slipRate_;
  float theta_ = 0.0f;     
  float encTheta_ = 0.0f;  
  bool slip_ = false;
};
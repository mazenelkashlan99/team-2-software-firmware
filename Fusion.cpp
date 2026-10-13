#include "Fusion.h"
#include "Angle.h"
#include <cmath>

void Fusion::reset(float theta) {
  theta_ = wrapPi(theta);
  encTheta_ = theta_;
  slip_ = false;
}

float Fusion::update(float dThetaEnc, float gz, float dt, bool imuValid) {
  const float prev = theta_;
  encTheta_ = wrapPi(encTheta_ + dThetaEnc);

  if (!imuValid || dt <= 0.0f) {          
    theta_ = wrapPi(theta_ + dThetaEnc);
    encTheta_ = theta_;
    slip_ = false;
    return wrapPi(theta_ - prev);
  }

  slip_ = std::fabs(gz - dThetaEnc / dt) > slipRate_;
  const float a = slip_ ? 1.0f : alpha_;  

  const float predicted = theta_ + gz * dt;
  const float err = wrapPi(encTheta_ - predicted);
  theta_ = wrapPi(predicted + (1.0f - a) * err);

  if (slip_) encTheta_ = theta_;
  return wrapPi(theta_ - prev);
}
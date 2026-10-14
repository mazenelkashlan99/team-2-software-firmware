#pragma once
#include <cmath>
#include "Angle.h"

struct WheelStep {
  float ds;      
  float dTheta;  
};

class Odometry {
public:
  static WheelStep wheelStep(float dL, float dR, float trackWidth) {
    return { (dR + dL) * 0.5f, (dR - dL) / trackWidth };
  }

  void reset(float x = 0.0f, float y = 0.0f, float theta = 0.0f) {
    x_ = x; y_ = y; theta_ = wrapPi(theta);
  }

  void integrate(float ds, float dTheta) {
    x_ += ds * std::cos(theta_ + dTheta * 0.5f);
    y_ += ds * std::sin(theta_ + dTheta * 0.5f);
    theta_ = wrapPi(theta_ + dTheta);
  }

  float x() const { return x_; }
  float y() const { return y_; }
  float theta() const { return theta_; }

private:
  float x_ = 0.0f, y_ = 0.0f, theta_ = 0.0f;
};
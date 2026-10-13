#pragma once
#include <cmath>

inline float wrapPi(float a) {
  const float PI_F = 3.14159265358979f;
  while (a > PI_F)  a -= 2.0f * PI_F;
  while (a < -PI_F) a += 2.0f * PI_F;
  return a;
}
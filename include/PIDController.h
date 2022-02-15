#pragma once
#include "vex.h"

class PID {

  public:

  PID(float kp, float ki, float kd);
  float tick(float error);

  private:

  float prevError = 0;
  float prevIntegral = 0;
  float K_p, K_i, K_d;

};
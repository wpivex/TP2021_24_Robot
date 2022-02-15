#pragma once
#include "vex.h"

static const int UNBOUNDED = 0;

class PID {

  public:

  PID(float kp, float ki, float kd, float bound = UNBOUNDED, float TOLERANCE = -1, float REPEATED = -1);
  float tick(float error);
  bool isCompleted();

  private:

  float prevError = 0;
  float prevIntegral = 0;
  float K_p, K_i, K_d;
  
  float TOLERANCE_THRESHOLD;
  int REPEATED_THRESHOLD;
  int repeated = 0;

  float BOUND;

};
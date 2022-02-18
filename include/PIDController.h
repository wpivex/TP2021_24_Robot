#pragma once
#include "vex.h"

static const int UNBOUNDED = 0;

struct PID_STRUCT {
  float p, i, d;
  float t, r;
};

static const struct PID_STRUCT DIST_24 = {1, 0, 0, 3, 10}; // going forward/curving for PID on stopping to target
static const struct PID_STRUCT GTURN_24 = {0.625, 0, 0.00625, 3, 10}; // gyro turn corrections, used both when going forward and turning to angle
static const struct PID_STRUCT VTURN_24 = {1, 0, 0, 3, 10}; // vision turn corrections, used both with vision forward and vision aling
static const struct PID_STRUCT ARM_FOUR = {0.3, 0, 0.3, 3, 10};
static const struct PID_STRUCT ARM_CHAIN = {0.35, 0, 0.1, 3, 10};


class PID {

  public:

  PID(float kp, float ki, float kd, float TOLERANCE = -1, float REPEATED = -1);
  PID(PID_STRUCT data);
  float tick(float error, float bound = UNBOUNDED);
  bool isCompleted();

  private:

  float prevError = 0;
  float prevIntegral = 0;
  float K_p, K_i, K_d;
  
  float TOLERANCE_THRESHOLD;
  int REPEATED_THRESHOLD;
  int repeated = 0;

};
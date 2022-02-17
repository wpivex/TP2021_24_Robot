#include "PIDController.h"
#include "constants.h"

PID::PID(float kp, float ki, float kd, float TOLERANCE, float REPEATED) {

  TOLERANCE_THRESHOLD = TOLERANCE; // the error threshold to be considered "done"
  REPEATED_THRESHOLD = REPEATED; // the number of times the threshold must be reached in a row to be considered "done"


  K_p = kp;
  K_i = ki;
  K_d = kd;
}

// bound is // max/min value of output. Useful for stuff like capping speed
float PID::tick(float error, float bound) {
  float integral = prevIntegral + error;
  float derivative = error - prevError;

  float output = K_p * error + K_i * integral + K_d * derivative;
  prevError = error;
  prevIntegral = integral;

  if (fabs(error) < TOLERANCE_THRESHOLD) repeated++;
  else repeated = 0;

  // If bound is not 0 (meaning unbounded), output should be clamped between -bound and +bound
  if (bound != UNBOUNDED) output = fmax(-bound, fmin(bound, output));

  logController("%f", output);
  return output;
}

PID::PID(PID_STRUCT data) {
  PID(data.p, data.i, data.d, data.t, data.r);
}

// Call to check whether PID has converged to a value. Use with stuff like arm movement and aligns/turns but not with stuff like driving straight
bool PID::isCompleted() {
  return repeated >= REPEATED_THRESHOLD;
}
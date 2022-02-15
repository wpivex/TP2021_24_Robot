#include "PIDController.h"

PID::PID(float kp, float ki, float kd, float bound, float TOLERANCE, float REPEATED) {

  TOLERANCE_THRESHOLD = TOLERANCE; // the error threshold to be considered "done"
  REPEATED_THRESHOLD = REPEATED; // the number of times the threshold must be reached in a row to be considered "done"

  BOUND = bound; // max/min value of output. Useful for stuff like capping speed

  K_p = kp;
  K_i = ki;
  K_d = kd;
}

float PID::tick(float error) {
  float integral = prevIntegral * 0.02;
  float derivative = (error - prevError) / 0.02;

  float output = K_p * error + K_i * integral + K_d * derivative;
  prevError = error;
  prevIntegral = integral;

  if (fabs(error) < TOLERANCE_THRESHOLD) repeated++;
  else repeated = 0;

  // If bound is not 0 (meaning unbounded), output should be clamped between -bound and +bound
  if (BOUND != UNBOUNDED) output = fmax(-BOUND, fmin(BOUND, output));

  return output;
}

// Call to check whether PID has converged to a value. Use with stuff like arm movement and aligns/turns but not with stuff like driving straight
bool PID::isCompleted() {
  return repeated >= REPEATED_THRESHOLD;
}
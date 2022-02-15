#include "PIDController.h"

PID::PID(float kp, float ki, float kd) {
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

  return output;
}
#include "TrapezoidalController.h"
#include "constants.h"


Trapezoid::Trapezoid(float distInches, float maxSpeedP, float minSpeedP, float rampUpInches, float slowDownInches) {

  dist = 0;
  direction = (distInches > 0 ? 1 : -1);
  finalDist = fabs(distInches);
  maxSpeed = maxSpeedP;
  minSpeed = minSpeedP;
  rampUp = rampUpInches;
  slowDown = slowDownInches;

  slowDown = fmin(slowDown, finalDist);
  rampUp = fmin(rampUp, finalDist);
  maxSpeed = fmax(minSpeed, maxSpeed);
  
}

float Trapezoid::get(float currDistance) {

  dist = fabs(currDistance);
  float delta;

  if (dist < rampUp) delta = dist / rampUp;
  else if (finalDist - dist < slowDown) delta = (finalDist - dist) / slowDown;
  else delta = 1;

  float speed = minSpeed + (maxSpeed - minSpeed) * delta;
  return direction * speed;
}

bool Trapezoid::isCompleted() {
  return dist >= finalDist;
}
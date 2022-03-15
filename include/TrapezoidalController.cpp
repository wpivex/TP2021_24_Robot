#include "TrapezoidController.h"

Trapezoid::Trapezoid(float initialP, float targetP, float maxSpeedP, float minSpeedP, float slowDownP, float slowEndP) {

  direction = initialP < targetP ? 1 : -1;
  target = targetP;
  maxSpeed = maxSpeedP;
  minSpeed = minSpeedP;
  slowDown = slowDownP;
  slowEnd = slowEndP;

  maxSpeed = fmax(minSpeed, maxSpeed); // make sure max speed is at least as fast as min speed
  
}

float Trapezoid::tick(float current) {

  float speed;

  if (fabs(target-current) < slowEnd) speed = minSpeed;
  else if (fabs(target-current) > slowDown) speed = maxSpeed;
  else speed = minSpeed + (maxSpeed - minSpeed) * fabs(target-current)) / slowDown;

  return speed * direction;
}

bool Trapezoid::isCompleted() {
  if (direction == 1) return current > target;
  return current < target;
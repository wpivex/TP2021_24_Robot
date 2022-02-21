#include "TrapezoidalController.h"
#include "constants.h"

// Regardless of total angle/distance, slowDownDegrees maintains a constant decrease in velocity
Trapezoid::Trapezoid(bool convertDistDegrees, float distP, float maxSpeedP, float minSpeedP, float rampUpP, float slowDownP, float endSlowP, float marginP) {

  dist = 0;
  direction = (distP > 0 ? 1 : -1);
  maxSpeed = maxSpeedP;
  minSpeed = minSpeedP;

  if (convertDistDegrees) {
    finalDist = fabs(distanceToDegrees(distP));
    rampUp = distanceToDegrees(rampUpP);
    slowDown = distanceToDegrees(slowDownP);
    endSlow = distanceToDegrees(endSlowP);
    margin = distanceToDegrees(marginP);
  } else {
    finalDist = distP;
    rampUp = rampUpP;
    slowDown = slowDownP;
    endSlow = endSlowP;
    margin = marginP;
  }

  rampUp = fmin(rampUp, finalDist);
  maxSpeed = fmax(minSpeed, maxSpeed);
  
}


float Trapezoid::get(float currDistance) {

  dist = fabs(currDistance);
  float delta;

  if (dist < rampUp) delta = dist / rampUp;
  else if (finalDist - dist < endSlow) delta = 0;
  else if (finalDist - dist < slowDown + endSlow) delta = (finalDist - dist) / slowDown;
  else delta = 1;

  float speed = minSpeed + (maxSpeed - minSpeed) * delta;
  return direction * speed;
}

bool Trapezoid::isCompleted() {
  return dist >= finalDist - margin;
}
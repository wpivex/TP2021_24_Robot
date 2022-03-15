#pragma once
#include "vex.h"
#include "constants.h"

class Trapezoid {

  public:
  Trapezoid(float initialP, float targetP, float maxSpeedP, float minSpeedP, float slowDownP, float slowEndP = 0);
  float tick(float current);
  bool isCompleted();
  
  private:

  float direction, target, maxSpeed, minSpeed, slowDown, slowEnd;

};
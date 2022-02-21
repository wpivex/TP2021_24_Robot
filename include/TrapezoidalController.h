#pragma once
#include "vex.h"

class Trapezoid {

  public:
  Trapezoid(bool convertDistDegrees, float distP, float maxSpeedP, float minSpeedP, float rampUpP, float slowDownP, float endSlowP = 0, float marginP = 0);
  float get(float currDistance);
  bool isCompleted();
  
  private:

  float direction;
  float finalDist, maxSpeed, minSpeed, rampUp, slowDown, endSlow, margin;
  float dist = 0;

};
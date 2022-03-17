#pragma once
#include "vex.h"

class Trapezoid {

  public:
  Trapezoid(float distInches, float maxSpeedP, float minSpeedP, float rampUpInches, float slowDownInches);
  float get(float currDistance);
  bool isCompleted();
  
  private:

  float direction;
  float finalDist, maxSpeed, minSpeed, rampUp, slowDown;
  float dist = 0;

};
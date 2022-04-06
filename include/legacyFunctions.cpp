#include "robot.h"
    // -------- Headers -----------
    void smartDrive(float distInches, float speed, directionType left, directionType right, int timeout, float slowDownInches, 
                    float turnPercent, bool stopAfter, std::function<bool(void)> func);
    void driveTurn(float degrees, float speed, bool isClockwise, int timeout, float slowDownInches = 10, 
                    bool stopAfter = true, std::function<bool(void)> func = {});
    void driveCurved(float distInches, float speed, directionType dir, int timeout, 
                      float slowDownInches, float turnPercent, bool stopAfter = true, std::function<bool(void)> func = {});
    void driveStraight(float distInches, float speed, directionType dir, int timeout, 
                      float slowDownInches, bool stopAfter = true, std::function<bool(void)> func = {});
    void driveStraightTimed(float speed, directionType dir, int timeMs, bool stopAfter = true, std::function<bool(void)> func = {});
    float goTurnVision2(Goal goal, directionType cameraDir, float minSpeed, float timeout);
    void driveStraightTimed(float speed, directionType dir, int timeout, bool stopAfter, std::function<bool(void)> func) {
      driveStraight(0, speed, dir, timeout, 0, stopAfter, func);
    void goTurnFastU(float universalAngleDegrees, float maxSpeed, float minSpeed, float slowDownDegrees, float endSlowDegrees = 0, 
      float timeout = 5, std::function<bool(void)> func = {});
    void goTurnFast(bool isClockwise, float turnDegrees, float maxSpeed, float minSpeed, float slowDownDegrees, float endSlowDegrees = 0,
      float timeout = 5, std::function<bool(void)> func = {});
}


void driveStraight(float distInches, float speed, directionType dir, int timeout, 
float slowDownInches, bool stopAfter, std::function<bool(void)> func) {

  driveCurved(distInches, speed, dir, timeout, slowDownInches, 0, stopAfter, func);

}

void driveCurved(float distInches, float speed, directionType dir, int timeout, 
float slowDownInches, float turnPercent, bool stopAfter, std::function<bool(void)> func) {

  smartDrive(distInches, speed, dir, dir, timeout, slowDownInches, turnPercent, stopAfter, func);

}

void driveTurn(float degrees, float speed, bool isClockwise, int timeout, float slowDownInches, 
bool stopAfter, std::function<bool(void)> func) {

  smartDrive(getTurnAngle(degrees), speed, isClockwise ? forward : reverse, isClockwise ? reverse: forward,
  timeout, slowDownInches, 0, stopAfter, func);

}

// distInches is positive distance in inches to destination. -1 means indefinite (until timeout)
// speed is percent 1-100
// direction is for left motor, right depends if turning
// timeout (optional parameter defaults to -1 -> none) in ms, terminates once reached
// slowDownInches representing from what distance to destination the robot starts slowing down with proportional speed
//  control in relation to distInches. Set by default to 10. 0 means attempt instant stop.
//  a higher value is more controlled/consistent, a lower value is faster/more variable
// turnPercent (from 0-1) is percent of speed to curve (so curvature now independent from speed). optional, default to 0
// stopAfter whether to stop motors after function call.
// func is an optional nonblocking function you can use to run as the same time as this method. It returns true when nonblocking function is gone
void smartDrive(float distInches, float speed, directionType left, directionType right, int timeout, 
float slowDownInches, float turnPercent, bool stopAfter, std::function<bool(void)> func) {


  float finalDist = distInches == 0? -1 : distanceToDegrees(distInches);
  float slowDown = distanceToDegrees(slowDownInches);

  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();

  // finalDist is 0 if we want driveTimed instead of drive some distance
  float currentDist = 0;
  while ((finalDist == -1 || currentDist < finalDist) && (timeout == -1 || vex::timer::system() < startTime + timeout*1000)) {

    // if there is a concurrent function to run, run it
    if (func) {
      if (func()) {
        // if func is done, make it empty
        func = {};
      }
    }

    currentDist = (fabs(leftMotorA.position(degrees)) + fabs(rightMotorA.position(degrees))) / 2;

     // from 0 to 1 indicating proportion of velocity. Starts out constant at 1 until it hits the slowDown interval,
     // where then it linearly decreases to 0
    float proportion = slowDown == 0 ? 1 : fmin(1, 1 - (currentDist - (finalDist - slowDown)) / slowDown);
    float baseSpeed = FORWARD_MIN_SPEED + (speed-FORWARD_MIN_SPEED) * proportion;

    //log("%f", baseSpeed);

    // turnPercent bounded between -1 (counterclockwise point turn) and 1 (clockwise point turn)
    float lspeed, rspeed;
    if (turnPercent >= 0) {
      lspeed = 1;
      rspeed = 1 - 2*turnPercent;
    } else {
      rspeed = 1;
      lspeed = 1 + 2*turnPercent;
    }

    setLeftVelocity(left, lspeed * baseSpeed);
    setRightVelocity(right, rspeed * baseSpeed);
    
    wait(20, msec);

  }

  if (stopAfter) {
    stopLeft();
    stopRight();
  }

  // log("done");

}

float goTurnVision2(Goal goal, directionType cameraDir, float minSpeed, float timeout) {

  const float END_SLOW_DEGREES = 0.5;
  const float SLOW_DOWN_DEGREES = 1; 
  const int MIN_SPEED = minSpeed;
  const int MAX_SPEED = 70;

  float delta, offset;
  int startTime = vex::timer::system();
  updateCamera(goal);

  vision *camera = (cameraDir == forward) ? &frontCamera : &backCamera;
  camera->takeSnapshot(goal.sig);

  while(!camera->largestObject.exists) {
    logController("Waiting for camera\n for %d seconds", vex::timer::system() - startTime);
    if(!isTimeout(startTime, timeout)) return -1;
    camera->takeSnapshot(goal.sig);
    wait(50, msec);
  }

  // false if overshooting to the right, true if overshooting to the left
  bool clockwise = camera->largestObject.centerX > VISION_CENTER_X;

  offset = 1;
  while (offset > 0.01 && !isTimeout(startTime, timeout)) {
    camera->takeSnapshot(goal.sig);
    if (camera->largestObject.exists)
       offset = (clockwise ? 1 : -1) * ((camera->largestObject.centerX - VISION_CENTER_X) / VISION_CENTER_X);
    if (offset < END_SLOW_DEGREES) delta = 0;
    else if (offset < SLOW_DOWN_DEGREES + END_SLOW_DEGREES) delta = (offset - END_SLOW_DEGREES) / SLOW_DOWN_DEGREES;
    else delta = 1;

    float speed = MIN_SPEED + delta * (MAX_SPEED - MIN_SPEED);

    logController("%f %f %f", offset, delta, speed);
    if (camera->largestObject.exists) {
      setLeftVelocity(clockwise ? forward : reverse, speed);
      setRightVelocity(clockwise ? reverse : forward, speed);
    }

    wait(20, msec);

  }

  stopLeft();
  stopRight();
  camera->takeSnapshot(goal.sig);
  if (camera->largestObject.exists)
    return (clockwise ? 1 : -1) * ((camera->largestObject.centerX - VISION_CENTER_X) / VISION_CENTER_X);
  else return -1;

}

// A fast but inaccurate turning function
void goTurnFast(bool isClockwise, float turnDegrees, float maxSpeed, float minSpeed, float slowDownDegrees, float endSlowDegrees, float timeout, std::function<bool(void)> func) {

  float delta;
  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();
  gyroSensor.resetRotation();
  slowDownDegrees = fmin(slowDownDegrees, turnDegrees);

  // logController("start turn fast");

  // Repeat until either arrived at target or timed out
  while (fabs(gyroSensor.rotation()) < turnDegrees && !isTimeout(startTime, timeout)) {

    if (func) {
      if (func()) {
        // if func is done, make it empty
        func = {};
      }
    }

    // logController("%f", gyroSensor.heading());

    float angle = fabs(gyroSensor.rotation());
    if (turnDegrees - angle < endSlowDegrees) delta = 0;
    else if (turnDegrees - angle < slowDownDegrees + endSlowDegrees) delta = (turnDegrees - endSlowDegrees - angle) / slowDownDegrees;
    else delta = 1;

    float speed = minSpeed + delta * (maxSpeed - minSpeed);

    setLeftVelocity(isClockwise ? forward : reverse, speed);
    setRightVelocity(isClockwise ? reverse : forward, speed);

    wait(20, msec);

  }
  // logController("finish turn fast");
  stopLeft();
  stopRight();

}

// Turn to some universal angle based on starting point. Turn direction is determined by smallest angle
void goTurnFastU(float universalAngleDegrees, float maxSpeed, float minSpeed, float slowDownDegrees, float endSlowDegrees, float timeout, 
std::function<bool(void)> func) {

  float gyroReading = gpsSensor.heading() + 180;//gyroSensor.heading(degrees);
  float turnAngle = universalAngleDegrees - gyroReading;
  if (turnAngle > 180) turnAngle -= 360;
  else if (turnAngle < -180) turnAngle += 360;

  logController("%f %f", turnAngle, gyroReading);
  goTurnFast(turnAngle > 0, fabs(turnAngle), maxSpeed, minSpeed, slowDownDegrees, endSlowDegrees, timeout, func);
}


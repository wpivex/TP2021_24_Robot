#include "robot.h"
#include <math.h>


// Motor ports Left: 1R, 2F, 3F,  20T Right: 12R, 11F, 13F
// gear ratio is 60/36
Robot::Robot(controller* c, bool _isSkills) : leftMotorA(0), leftMotorB(0), leftMotorC(0), leftMotorD(0), leftMotorE(0), rightMotorA(0), rightMotorB(0), 
  rightMotorC(0), rightMotorD(0), rightMotorE(0), fourBarLeft(0), fourBarRight(0), chainBarLeft(0), chainBarRight(0), claw(0), frontCamera(0), 
  backCamera(0), gyroSensor(PORT16) {

  isSkills = _isSkills;

  leftMotorA = motor(PORT1, ratio18_1, true); 
  leftMotorB = motor(PORT2, ratio18_1, true);
  leftMotorC = motor(PORT3, ratio18_1, true);
  leftMotorD = motor(PORT4, ratio18_1, true);
  leftMotorE = motor(PORT5, ratio18_1, true);
  leftDrive = motor_group(leftMotorA, leftMotorB, leftMotorC, leftMotorD, leftMotorE);

  rightMotorA = motor(PORT11, ratio18_1, false);
  rightMotorB = motor(PORT12, ratio18_1, false);
  rightMotorC = motor(PORT13, ratio18_1, false);
  rightMotorD = motor(PORT14, ratio18_1, false);
  rightMotorE = motor(PORT15, ratio18_1, false);
  rightDrive = motor_group(rightMotorA, rightMotorB, rightMotorC, rightMotorD, rightMotorE);

  fourBarLeft = motor(PORT20, ratio18_1, false);
  fourBarRight = motor(PORT17, ratio18_1, true);
  chainBarLeft = motor(PORT10, ratio18_1, false);
  chainBarRight = motor(PORT18, ratio18_1, true);
  claw = motor(PORT19, ratio18_1, false);

  YELLOW_SIG = new vision::signature (1, 1849, 2799, 2324, -3795, -3261, -3528, 2.500, 0);
  RED_SIG = new vision::signature (1, 5767, 9395, 7581, -685, 1, -342, 3.000, 0);
  BLUE_SIG = new vision::signature (1, -2675, -1975, -2324, 8191, 14043, 11116, 3.000, 0);

  driveType = ARCADE;
  robotController = c; 
  frontCamera = vision(PORT9, 50, *YELLOW_SIG);
  backCamera = vision(PORT8, 50, *YELLOW_SIG);

  fourBarLeft.setBrake(hold);
  fourBarRight.setBrake(hold);
  chainBarLeft.setBrake(hold);
  chainBarRight.setBrake(hold);
  claw.setBrake(hold);

}

void Robot::driveTeleop() {
  float leftVert = (float) robotController->Axis3.position();
  float rightVert = (float) robotController->Axis2.position();
  float rightHoriz = (pow((float) robotController->Axis1.position()/100.0, 3)*100.0);

  if(driveType == ARCADE) {
    float left;
    float right;
    if(rightHoriz < 0) {
      left = leftVert + pow(rightHoriz, 3);
      right = leftVert - pow(rightHoriz, 3);
    } else {
      left = leftVert + pow(rightHoriz, 3);
      right = leftVert - pow(rightHoriz, 3);
    }

    setLeftVelocity(forward, left/fabs(left)*fmin(fabs(left), 100));
    setRightVelocity(forward, right/fabs(right)*fmin(fabs(right), 100));      
  } else {
    setLeftVelocity(forward, leftVert * 100);
    setRightVelocity(forward,  rightVert * 100);      
  }
}


void Robot::goalClamp() {
  if (Robot::robotController->ButtonL1.pressing()) {
    if(!frontWasPressed) {
      frontGoal.set(!frontGoal.value());
    }
    frontWasPressed = true;
  } else {
    frontWasPressed = false;
  }

  if (Robot::robotController->ButtonR1.pressing()) {
    if(!backWasPressed) {
      backGoal.set(!backGoal.value());
    }
    backWasPressed = true;
  } else {
    backWasPressed = false;
  }
}

void Robot::clawMovement() {
  if (Robot::robotController->ButtonUp.pressing()) {
    if(!clawWasPressed) {
      claw.rotateTo(isClawOpen? 500 : MAX_CLAW, deg, 100, velocityUnits::pct, false);
      isClawOpen = !isClawOpen;
    }
    clawWasPressed = true;
  } else {
    clawWasPressed = false;
  }
}

void Robot::setFrontClamp(bool intaking) {
  frontGoal.set(intaking);
}

void Robot::setBackClamp(bool intaking) {
  backGoal.set(intaking);
}

// Run every tick
void Robot::teleop() {
  driveTeleop();
  //armMovement(true, 100);
  clawMovement();
  goalClamp();
  wait(20, msec);
}





void Robot::driveStraightTimed(float speed, directionType dir, int timeMs, bool stopAfter, std::function<bool(void)> func) {
  driveStraight(0, speed, dir, timeMs, 1, stopAfter, func);
}


void Robot::driveStraight(float distInches, float speed, directionType dir, int timeout, 
float slowDownInches, bool stopAfter, std::function<bool(void)> func) {

  log("driving straight");
  driveCurved(distInches, speed, dir, timeout, slowDownInches, 0, stopAfter, func);

}

void Robot::driveCurved(float distInches, float speed, directionType dir, int timeout, 
float slowDownInches, float turnPercent, bool stopAfter, std::function<bool(void)> func) {

  smartDrive(distInches, speed, dir, dir, timeout, slowDownInches, turnPercent, stopAfter, func);

}

void Robot::driveTurn(float degrees, float speed, bool isClockwise, int timeout, float slowDownInches, 
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
void Robot::smartDrive(float distInches, float speed, directionType left, directionType right, int timeout, 
float slowDownInches, float turnPercent, bool stopAfter, std::function<bool(void)> func) {


  float finalDist = distanceToDegrees(distInches);
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

    log("%f", baseSpeed);

    // reduce baseSpeed so that the faster motor always capped at max speed
    baseSpeed = fmin(baseSpeed, 100 - baseSpeed*turnPercent);

    setLeftVelocity(left, baseSpeed*(1 + turnPercent));
    setRightVelocity(right, baseSpeed*(1 - turnPercent));
    
    wait(20, msec);

  }

  if (stopAfter) {
    stopLeft();
    stopRight();
  }

  log("done");

}

// Move forward/backward with proportional gyro feedback.
// finalDegrees is the delta yaw angle at the end of the curve
void Robot::driveStraightGyro(float distInches, float speed, directionType dir, int timeout, float slowDownInches, std::function<bool(void)> func) {

  float finalDist = distanceToDegrees(distInches);
  float slowDown = distanceToDegrees(slowDownInches);


  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();
  gyroSensor.resetRotation();

  const float GYRO_CONSTANT = 0.01;

  // finalDist is 0 if we want driveTimed instead of drive some distance
  float currentDist = 0;
  while ((finalDist == 0 || currentDist < finalDist) && !isTimeout(startTime, timeout)) {

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

    float gyroCorrection = gyroSensor.rotation() * GYRO_CONSTANT;


    // reduce baseSpeed so that the faster motor always capped at max speed
    baseSpeed = fmin(baseSpeed, 100 - baseSpeed*gyroCorrection);

    float left = baseSpeed*(1 - gyroCorrection);
    float right = baseSpeed*(1 + gyroCorrection);
    setLeftVelocity(dir, left);
    setRightVelocity(dir, right);
    log("%f %f %f", gyroCorrection, left, right);
    
    wait(20, msec);
  }

  stopLeft();
  stopRight();
  
}

// BOTH angleDegrees AND startSlowDownDegrees SHOULD BE POSITIVE
// angleDegrees indicates the angle to turn to.
// startSlowDownDegrees is some absolute angle less than angleDegrees, where gyro proportional control starts
// maxSpeed is the starting speed of the turn. Will slow down once past startSlowDownDegrees theshhold
void Robot::turnToAngleGyro(bool clockwise, float angleDegrees, float maxSpeed, int startSlowDownDegrees,
int timeout, std::function<bool(void)> func) {


  float speed;

  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();
  gyroSensor.resetRotation();

  float currDegrees = 0; // always positive with abs

  while (currDegrees < angleDegrees && !isTimeout(startTime, timeout)) {

    // if there is a concurrent function to run, run it
    if (func) {
      if (func()) {
        // if func is done, make it empty
        func = {};
      }
    }

    currDegrees = float(gyroSensor.rotation());
    if (currDegrees < startSlowDownDegrees) {
      // before hitting theshhold, speed is constant at starting speed
      speed = maxSpeed;
    } else {
      float delta = (angleDegrees - currDegrees) / (angleDegrees - startSlowDownDegrees); // starts at 1 @deg=startSlowDeg, becomes 0 @deg = final
      speed = TURN_MIN_SPEED + delta * (speed - TURN_MIN_SPEED);
    }

    setLeftVelocity(clockwise ? forward : reverse, speed);
    setRightVelocity(clockwise ? reverse : forward, speed);
  }

  stopLeft();
  stopRight();
}


// Go forward until the maximum distance is hit, the timeout is reached, or limitSwitch is turned on (collision with goal)
// for indefinite timeout, set to -1
void Robot::goForwardVision(Goal goal, float speed, directionType dir, float maxDistanceInches, int timeout, 
digital_in* limitSwitch, std::function<bool(void)> func) {

  // The proportion to turn in relation to how offset the goal is. Is consistent through all speeds
  const float PMOD_MULTIPLIER = 1.2;

  int pMod = speed * PMOD_MULTIPLIER;
  float baseSpeed = fmin(speed, 100 - pMod);

  float totalDist = distanceToDegrees(maxDistanceInches);
  float dist = 0;

  vision *camera = (dir == forward) ? &frontCamera : &backCamera;
  camera->setBrightness(goal.bright);

  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();

  int sign = (dir == forward) ? 1 : -1;

  // forward until the maximum distance is hit, the timeout is reached, or limitSwitch is turned on
  while (dist < totalDist && !isTimeout(startTime, timeout) && (limitSwitch == nullptr || !limitSwitch->value())) {

    // if there is a concurrent function to run, run it
    if (func) {
      if (func()) {
        // if func is done, make it empty
        func = {};
      }
    }

    camera->takeSnapshot(goal.sig);
    
    if(camera->largestObject.exists) {

      float mod = pMod * VISION_CENTER_X-camera->largestObject.centerX / VISION_CENTER_X;
      setLeftVelocity(dir, baseSpeed - mod*sign);
      setRightVelocity(dir, baseSpeed + mod*sign);
    }

    wait(20, msec);
    dist = fabs((leftMotorA.position(degrees) + rightMotorA.position(degrees)) / 2.0);
  }

  stopLeft();
  stopRight();
  
}

void Robot::alignToGoalVision(Goal goal, bool clockwise, directionType cameraDirection, int timeout) {

  // spin speed is proportional to distance from center, but must be bounded between MIN_SPEED and MAX_SPEED
  const float MAX_SPEED = 40;

  vision *camera = (cameraDirection == forward) ? &frontCamera : &backCamera;
  camera->setBrightness(goal.bright);

  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();

  // At the initial snapshot we check where goal is seen. If so, we set our direction to be towards the goal and stop when we pass the center.
  // Otherwise, the spin will default to the direction specified by the 'clockwise' parameter
  frontCamera.takeSnapshot(goal.sig);
  if (frontCamera.largestObject.exists) {
    clockwise = (frontCamera.largestObject.centerX / VISION_CENTER_X) > VISION_CENTER_X;
  }

  int spinSign = clockwise ? -1 : 1;

  while (!isTimeout(startTime, timeout)) {

    frontCamera.takeSnapshot(goal.sig);

    float mod;
    if (frontCamera.largestObject.exists) {
      mod = VISION_CENTER_X-frontCamera.largestObject.centerX / VISION_CENTER_X;

      // If goal is [left side of screen if clockwise, right side of scree if counterclockwise], that means it's arrived at center and is aligned
      if (mod * spinSign >= 0) break;

    } else {
      // If largest object not detected, then spin in the specified direction
      mod = spinSign;
    }

    float speed = (mod > 0 ? 1 : -1) * TURN_MIN_SPEED + mod * (MAX_SPEED - TURN_MIN_SPEED);

    setLeftVelocity(reverse, speed);
    setRightVelocity(forward, speed);

  }
}


void Robot::openClaw() {
  claw.rotateTo(MAX_CLAW, deg, 100, velocityUnits::pct);
  isClawOpen = true;
}

void Robot::closeClaw() {
  int clawTimeout = vex::timer::system();
  while(true) {
    if (vex::timer::system() - clawTimeout > 1000) { return; }
    claw.rotateTo(500, deg, 100, velocityUnits::pct, false);
    wait(20, msec);
  }
  isClawOpen = false;
}


void Robot::setLeftVelocity(directionType d, double percent) {
  leftMotorA.spin(d, percent, pct);
  leftMotorB.spin(d, percent, pct);
  leftMotorC.spin(d, percent, pct);
  leftMotorD.spin(d, percent, pct);
  leftMotorE.spin(d, percent, pct);
}

void Robot::setRightVelocity(directionType d, double percent) {
  rightMotorA.spin(d, percent, pct);
  rightMotorB.spin(d, percent, pct);
  rightMotorC.spin(d, percent, pct);
  rightMotorD.spin(d, percent, pct);
  rightMotorE.spin(d, percent, pct);
}

void Robot::stopLeft() {
  leftMotorA.stop();
  leftMotorB.stop();
  leftMotorC.stop();
  leftMotorD.stop();
  leftMotorE.stop();
}

void Robot::stopRight() {
  rightMotorA.stop();
  rightMotorB.stop();
  rightMotorC.stop();
  rightMotorD.stop();
  rightMotorE.stop();
}

// log output to brain display the way you would with printf
template <class ... Args>
void Robot::log(const char *format, Args ... args) {

  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print(format, args...);

}
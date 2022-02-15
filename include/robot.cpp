#include "robot.h"
#include <math.h>


// Motor ports Left: 1R, 2F, 3F,  20T Right: 12R, 11F, 13F
// gear ratio is 60/36
Robot::Robot(controller* c, bool _isSkills) : leftMotorA(0), leftMotorB(0), leftMotorC(0), leftMotorD(0), leftMotorE(0), rightMotorA(0), rightMotorB(0), 
  rightMotorC(0), rightMotorD(0), rightMotorE(0), fourBarLeft(0), fourBarRight(0), chainBarLeft(0), chainBarRight(0), claw(0), frontCamera(PORT13), 
  backCamera(PORT15), gyroSensor(PORT4), arm(), buttons(c) {

    log("hello");

  isSkills = _isSkills;

  leftMotorA = motor(PORT1, ratio18_1, true); 
  leftMotorB = motor(PORT2, ratio18_1, true);
  leftMotorC = motor(PORT3, ratio18_1, true);
  leftMotorD = motor(PORT5, ratio18_1, true);
  leftMotorE = motor(PORT6, ratio18_1, true);
  leftDrive = motor_group(leftMotorA, leftMotorB, leftMotorC, leftMotorD, leftMotorE);

  rightMotorA = motor(PORT16, ratio18_1, false);
  rightMotorB = motor(PORT17, ratio18_1, false);

  rightMotorC = motor(PORT18, ratio18_1, false);
  rightMotorD = motor(PORT19, ratio18_1, false);
  rightMotorE = motor(PORT20, ratio18_1, false);
  rightDrive = motor_group(rightMotorA, rightMotorB, rightMotorC, rightMotorD, rightMotorE);

  fourBarLeft = motor(PORT10, ratio18_1, false);
  fourBarRight = motor(PORT14, ratio18_1, true);
  chainBarLeft = motor(PORT8, ratio18_1, false);
  chainBarRight = motor(PORT7, ratio18_1, true);
  claw = motor(PORT12, ratio18_1, false);

  gyroSensor = inertial(PORT4);

  arm.init(&buttons, chainBarLeft, chainBarRight, fourBarLeft, fourBarRight);

  driveType = TWO_STICK_ARCADE;
  robotController = c; 

  fourBarLeft.setBrake(hold);
  fourBarRight.setBrake(hold);
  chainBarLeft.setBrake(hold);
  chainBarRight.setBrake(hold);
  claw.setBrake(hold);

  setControllerMapping(DEFAULT_MAPPING);

}

void Robot::setControllerMapping(ControllerMapping mapping) {

  cMapping = mapping;

  if (mapping == DEFAULT_MAPPING) {
    driveType = TWO_STICK_ARCADE;

    FRONT_CLAMP_TOGGLE = Buttons::L1;
    BACK_CLAMP_TOGGLE = Buttons::R1;
    CLAW_TOGGLE = Buttons::UP;
  } 

}


void Robot::driveTeleop() {

  if(driveType == TANK) {
    setLeftVelocity(forward,buttons.axis(Buttons::LEFT_VERTICAL));
    setRightVelocity(forward,buttons.axis(Buttons::RIGHT_VERTICAL));
  }else{
    float drive = driveType == ONE_STICK_ARCADE ? buttons.axis(Buttons::RIGHT_VERTICAL) : buttons.axis(Buttons::LEFT_VERTICAL);
    float turn = buttons.axis(Buttons::RIGHT_HORIZONTAL);
    float max = std::max(1.0, std::max(fabs(drive+turn), fabs(drive-turn)));
    setLeftVelocity(forward,100 * (drive+turn)/max);
    setRightVelocity(forward,100 * (drive-turn)/max);
  }
}


void Robot::goalClamp() {

  if (buttons.pressed(FRONT_CLAMP_TOGGLE)) {
    frontGoal.set(!frontGoal.value());
  } else if (buttons.pressed(BACK_CLAMP_TOGGLE)) {
    backGoal.set(!backGoal.value());
  }
}

void Robot::clawMovement() {
  if (buttons.pressed(CLAW_TOGGLE)) {
    claw.rotateTo(isClawOpen? 2000 : MAX_CLAW, deg, 100, velocityUnits::pct, false);
      isClawOpen = !isClawOpen;
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
  arm.armMovement(true, 60);
  clawMovement();
  goalClamp();

  buttons.updateButtonState();

}

void Robot::callibrateGyro() {
  gyroSensor.calibrate();
  gyroSensor.resetHeading();
  while (gyroSensor.isCalibrating()) wait(20, msec);
}


void Robot::driveStraightTimed(float speed, directionType dir, int timeout, bool stopAfter, std::function<bool(void)> func) {
  driveStraight(0, speed, dir, timeout, 0, stopAfter, func);
}


void Robot::driveStraight(float distInches, float speed, directionType dir, int timeout, 
float slowDownInches, bool stopAfter, std::function<bool(void)> func) {

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

    // reduce baseSpeed so that the faster motor always capped at max speed
    baseSpeed = fmin(baseSpeed, 100 - baseSpeed*fabs(turnPercent));

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

// Go forward towards some universal direction
// Trapezoidal motion profiling with rampUp and slowDown.
// rampUp defaults to 0; slowDown defaults to the entire run after rampUp
// Two PID controllers used for left/right correction going straight, and for slowing down to target
// Linear increase in speed for rampUpInches
// distInches should be positive for forwards, negative for reverse
void Robot::goForwardU(float distInches, float maxSpeed, float uDirection, float rampUpInches, int timeout, std::function<bool(void)> func) {


  float rampUp = distanceToDegrees(rampUpInches);
  float finalDist = distanceToDegrees(distInches);

  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();

  // Set gyro direction 
  float correction = gyroSensor.heading() - uDirection;
  if (correction > 180) correction -= 360;
  if (correction < -180) correction += 360;
  gyroSensor.setRotation(correction, degrees);

  PID anglePID(GTURN_24);
  PID distPID(DIST_24);

  float currentDist, speed;


  // Repeat until either arrived at target or timed out
  while (!distPID.isCompleted() && !isTimeout(startTime, timeout)) {

    // if there is a concurrent function to run, run it
    if (func) {
      bool done = func();
      if (done) {
        // if func is done, make it empty
        func = {};
      }
    }

    currentDist = (leftMotorA.position(degrees) + rightMotorA.position(degrees)) / 2;

    if (fabs(currentDist) < rampUp) { // initial acceleration
      speed = maxSpeed * (fabs(currentDist) / rampUp);
      logController("ramp up");
    } else {
      speed = distPID.tick(finalDist - currentDist, maxSpeed);
      logController("PID");
    }
    
    float angleCorrection = anglePID.tick(gyroSensor.rotation());

    setLeftVelocity(forward, speed + angleCorrection);
    setRightVelocity(forward, speed - angleCorrection);

    wait(20, msec);
  }
  logController("done");
  stopLeft();
  stopRight();

}

// Go forward in whatever direction it was already in
void Robot::goForward(float distInches, float maxSpeed, float rampUpInches, int timeout, std::function<bool(void)> func) {

  goForwardU(distInches, maxSpeed, gyroSensor.heading(), rampUpInches, timeout, func);

}

// Curve forward/backward with PID to target
// Does not use gyro sensor. 
// distInches is positive if forward, negative if reverse
void Robot::goCurve(float distInches, float maxSpeed, float turnPercent, float rampUpInches, int timeout, std::function<bool(void)> func) {

  float rampUp = distanceToDegrees(rampUpInches);
  float finalDist = distanceToDegrees(distInches);

  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();

  PID distPID(DIST_24);

  float currentDist, speed;


  // Repeat until either arrived at target or timed out
  while (!distPID.isCompleted() && !isTimeout(startTime, timeout)) {

    // if there is a concurrent function to run, run it
    if (func) {
      bool done = func();
      if (done) {
        // if func is done, make it empty
        func = {};
      }
    }

    currentDist = (leftMotorA.position(degrees) + rightMotorA.position(degrees)) / 2;

    if (fabs(currentDist) < rampUp) { // initial acceleration
      speed = maxSpeed * (fabs(currentDist) / rampUp);
      logController("ramp up");
    } else {
      speed = distPID.tick(finalDist - currentDist, maxSpeed);
      logController("PID");
    }
    

    setLeftVelocity(forward, speed * (1 + turnPercent));
    setRightVelocity(forward, speed * (1 + turnPercent));

    wait(20, msec);
  }
  logController("done");
  stopLeft();
  stopRight();

}

vision* Robot::getCamera(directionType dir, Goal goal) {
  vision* camera = (dir == forward) ? &frontCamera : &backCamera;
  camera->setBrightness(goal.bright);
  camera->setSignature(goal.sig);
  return camera;
}

void Robot::goVision(float distInches, float maxSpeed, float rampUpInches, Goal goal, directionType cameraDir, int timeout, std::function<bool(void)> func) {

  vision *camera = getCamera(cameraDir, goal);

  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();

  PID vPID(VTURN_24);

  // still writing pls hold

}



// Go forward until the maximum distance is hit, the timeout is reached, or limitSwitch is turned on (collision with goal)
// for indefinite timeout, set to -1
void Robot::goForwardVision(Goal goal, float speed, directionType dir, float maxDistanceInches, int timeout, 
digital_in* limitSwitch, std::function<bool(void)> func, float pModMult) {

  int pMod = speed * pModMult;
  float baseSpeed = fmin(speed, 100 - pMod);

  float totalDist = distanceToDegrees(maxDistanceInches);
  float dist = 0;

  updateCamera(goal);

  vision *camera = (dir == forward) ? &frontCamera : &backCamera;

  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();

  int sign = (dir == forward) ? 1 : -1;

  // forward until the maximum distance is hit, the timeout is reached, or limitSwitch is turned on
  while (dist < totalDist && !isTimeout(startTime, timeout) && (limitSwitch == nullptr || !limitSwitch->value())) {
    // log("Start of loop");
    // if there is a concurrent function to run, run it
    if (func) {
      if (func()) {
        // if func is done, make it empty
        func = {};
      }
    }

    // log("Before snapshot");
    camera->takeSnapshot(goal.sig);
    
    if(camera->largestObject.exists) {
      log("%d", camera->largestObject.centerX);
      float mod = pMod * (VISION_CENTER_X-camera->largestObject.centerX) / VISION_CENTER_X;
      setLeftVelocity(dir, baseSpeed - mod*sign);
      setRightVelocity(dir, baseSpeed + mod*sign);
    }

    wait(20, msec);
    dist = fabs((leftMotorA.position(degrees) + rightMotorA.position(degrees)) / 2.0);
  }

  stopLeft();
  stopRight();
  
}


void Robot::gyroTurn(bool clockwise, float angleDegrees) {

  float K_PROPORTIONAL = 0.625;
  float K_DERIVATIVE = 0.00625;
  float tolerance = 3;

  float timeout = 5;
  float speed;

  log("initing");
  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();
  gyroSensor.resetRotation();
  log("about to loop");

  float currDegrees = 0; // always positive with abs
  float delta = 0;
  float delta_prev = 0;
  float delta_dir = 0;

  int NUM_VALID_THRESHOLD = 10;
  int numValid = 0;

  while (numValid < NUM_VALID_THRESHOLD && !isTimeout(startTime, timeout)) {

    currDegrees = fabs(gyroSensor.rotation());

    delta = angleDegrees - currDegrees;
    delta_dir = (delta - delta_prev)/0.02;

    speed = delta * K_PROPORTIONAL + delta_dir * K_DERIVATIVE;
    

    //logController("%d %f %f", clockwise? 1:0, speed, delta);
    logController("%f %f", delta*K_PROPORTIONAL, delta_dir*K_DERIVATIVE);
    setLeftVelocity(clockwise ? forward : reverse, speed);
    setRightVelocity(clockwise ? reverse : forward, speed);

    delta_prev = delta;

    wait(20, msec);
    if (fabs(currDegrees - angleDegrees) < tolerance) numValid++;
    else numValid = 0;
    //logController("%d %f", numValid, delta);
  }

  stopLeft();
  stopRight();
}

void Robot::gyroTurnU(float universalAngleDegrees) {
  float universalHeading = gyroSensor.heading(degrees);
  float turnAngle = fabs(universalHeading-universalAngleDegrees);
  bool clockwise = universalHeading < universalAngleDegrees;
  if (turnAngle > 180) {
    clockwise = !clockwise;
    turnAngle = 360 - turnAngle;
  }
  gyroTurn(clockwise, turnAngle);
}

void Robot::alignToGoalVision(Goal goal, bool clockwise, directionType cameraDirection, int timeout, float maxSpeed) {

  // spin speed is proportional to distance from center, but must be bounded between MIN_SPEED and MAX_SPEED

  updateCamera(goal);
  vision *camera = (cameraDirection == forward) ? &frontCamera : &backCamera;

  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();

  // // At the initial snapshot we check where goal is seen. If so, we set our direction to be towards the goal and stop when we pass the center.
  // // Otherwise, the spin will default to the direction specified by the 'clockwise' parameter
  // camera->takeSnapshot(goal.sig);
  // if (camera->largestObject.exists) {
  //   clockwise = (camera->largestObject.centerX / VISION_CENTER_X) > VISION_CENTER_X;
  // }

  int spinSign = clockwise ? -1 : 1;

  while (!isTimeout(startTime, timeout)) {

    camera->takeSnapshot(goal.sig);

    float mod;
    if (camera->largestObject.exists) {
      // log("%d", camera->largestObject.centerX);
      mod = (VISION_CENTER_X-camera->largestObject.centerX) / VISION_CENTER_X;

      // If goal is [left side of screen if clockwise, right side of scree if counterclockwise], that means it's arrived at center and is aligned
      if (fabs(mod) <= 0.05) {
        break;
      }

    } else {
      // If largest object not detected, then spin in the specified direction
      mod = spinSign;
    }

    float speed = (mod > 0 ? 1 : -1) * TURN_MIN_SPEED + mod * (maxSpeed - TURN_MIN_SPEED);

    setLeftVelocity(reverse, speed);
    setRightVelocity(forward, speed);
    wait(20, msec);
  }

  log("D0ne");

  stopLeft();
  stopRight();
}


void Robot::openClaw() {
  int clawTimeout = vex::timer::system();
  while(vex::timer::system() - clawTimeout < 2000 && claw.rotation(degrees) > MAX_CLAW) {
    claw.rotateTo(MAX_CLAW, deg, 100, velocityUnits::pct, false);
    wait(20, msec);
  }
  isClawOpen = true;
}

void Robot::closeClaw() {
  int clawTimeout = vex::timer::system();
  while(true) {
    if (vex::timer::system() - clawTimeout > 2000) { return; }
    claw.rotateTo(2000, deg, 100, velocityUnits::pct, false);
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

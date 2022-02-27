#include <math.h>
#include "robot.h"


// Motor ports Left: 1R, 2F, 3F,  20T Right: 12R, 11F, 13F
// gear ratio is 60/36
Robot::Robot(controller* c, bool _isSkills) : leftMotorA(0), leftMotorB(0), leftMotorC(0), leftMotorD(0), leftMotorE(0), rightMotorA(0), rightMotorB(0), 
  rightMotorC(0), rightMotorD(0), rightMotorE(0), fourBarLeft(0), fourBarRight(0), chainBarLeft(0), chainBarRight(0), frontCamera(PORT10), 
  backCamera(PORT15), gyroSensor(PORT4), arm(), buttons(c) {

    log("hello");

  isSkills = _isSkills;

  leftMotorA = motor(PORT1, ratio18_1, true); 
  leftMotorB = motor(PORT6, ratio18_1, true);
  leftMotorC = motor(PORT3, ratio18_1, true);
  leftMotorD = motor(PORT4, ratio18_1, true);
  leftMotorE = motor(PORT5, ratio18_1, true);
  leftDrive = motor_group(leftMotorA, leftMotorB, leftMotorC, leftMotorD, leftMotorE);

  rightMotorA = motor(PORT16, ratio18_1, false);
  rightMotorB = motor(PORT17, ratio18_1, false);

  rightMotorC = motor(PORT18, ratio18_1, false);
  rightMotorD = motor(PORT19, ratio18_1, false);
  rightMotorE = motor(PORT20, ratio18_1, false);
  rightDrive = motor_group(rightMotorA, rightMotorB, rightMotorC, rightMotorD, rightMotorE);

  fourBarLeft = motor(PORT7, ratio18_1, false);
  fourBarRight = motor(PORT14, ratio18_1, true);
  chainBarLeft = motor(PORT9, ratio18_1, false);
  chainBarRight = motor(PORT8, ratio18_1, true);

  gyroSensor = inertial(PORT11);

  arm.init(&buttons, chainBarLeft, chainBarRight, fourBarLeft, fourBarRight, chainBarPot, fourBarBump);

  driveType = TWO_STICK_ARCADE;
  robotController = c; 

  fourBarLeft.setBrake(hold);
  fourBarRight.setBrake(hold);
  chainBarLeft.setBrake(hold);
  chainBarRight.setBrake(hold);

  setControllerMapping(DEFAULT_MAPPING);

}

void Robot::setControllerMapping(ControllerMapping mapping) {

  cMapping = mapping;

  if (mapping == DEFAULT_MAPPING) {
    driveType = TWO_STICK_ARCADE;

    FRONT_CLAMP_TOGGLE = Buttons::L1;
    BACK_CLAMP_TOGGLE = Buttons::R1;
    CLAW_TOGGLE = Buttons::UP;
    ARM_TOGGLE = Buttons::R2;
  } 

}


void Robot::driveTeleop() {

  if (armHold) setBrakeType(hold);
  else setBrakeType(coast);

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
    if(testingArm) {
      printf("{%f, %f},\n",  fourBarRight.position(degrees), chainBarPot.value(deg));
    } else {
      frontGoal.set(!frontGoal.value());
    }
  } else if (buttons.pressed(BACK_CLAMP_TOGGLE)) {
    if(testingArm) {
      armCoast = !armCoast;
      fourBarLeft.setBrake(armCoast? coast : hold);
      fourBarRight.setBrake(armCoast? coast : hold);
      chainBarLeft.setBrake(armCoast? coast : hold);
      chainBarRight.setBrake(armCoast? coast : hold);
    } else {
      backGoal.set(!backGoal.value());
    }
  }
}

void Robot::clawMovement() {
  if (buttons.pressed(CLAW_TOGGLE)) {
    clawPiston.set(!clawPiston.value());
  }
}

void Robot::setFrontClamp(bool intaking) {
  frontGoal.set(intaking);
}

void Robot::setBackClamp(bool intaking) {
  backGoal.set(intaking);
}

void Robot::armTeleop() {

  if (armHold) setMaxArmTorque(ARM_CURRENT::LOW);
  else {
    if (arm.isMoving()) setMaxArmTorque(ARM_CURRENT::HIGH);
    else setMaxArmTorque(ARM_CURRENT::MID);
  }

  if(vex::controller().ButtonY.pressing()) {
    // "Disable steppers" except you gotta hold it down
    fourBarLeft.setBrake(coast);
    fourBarRight.setBrake(coast);
    chainBarLeft.setBrake(coast);
    chainBarRight.setBrake(coast);
    fourBarLeft.stop();
    fourBarRight.stop();
  } else {
    fourBarLeft.setBrake(hold);
    fourBarRight.setBrake(hold);
    chainBarLeft.setBrake(hold);
    chainBarRight.setBrake(hold);
    arm.armMovement(true, 100);
  }
  const float l = fourBarLeft.rotation(vex::degrees), r = fourBarLeft.rotation(vex::degrees);
  const float ll = chainBarLeft.rotation(vex::degrees), rr = chainBarRight.rotation(vex::degrees);
  log(6, "Live Vals:");
  log(7, "FBL %4f   FBR: %4f",l, r);
  log(8, "CBL %4f   CBR: %4f",ll, rr);

}

// Run every tick
void Robot::teleop() {

  if (buttons.pressed(ARM_TOGGLE)) {
      armHold = !armHold;
  }

  driveTeleop();
  armTeleop();
  clawMovement();
  goalClamp();
  // logController("%f", leftMotorA.current(currentUnits::amp));

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

  log("done");

}



// Go forward in whatever direction it was already in
// Trapezoidal motion profiling
void Robot::goForward(float distInches, float maxSpeed, float rampUpInches, float slowDownInches, int timeout, std::function<bool(void)> func) {

  Trapezoid trap(distInches, maxSpeed, FORWARD_MIN_SPEED, rampUpInches, slowDownInches);

  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();
  gyroSensor.resetRotation();

  // finalDist is 0 if we want driveTimed instead of drive some distance
  while (!trap.isCompleted() && !isTimeout(startTime, timeout)) {

    // if there is a concurrent function to run, run it
    if (func) {
      bool done = func();
      if (done) {
        // if func is done, make it empty
        func = {};
      }
    }

    float speed = trap.get( (leftMotorA.position(degrees) + rightMotorA.position(degrees)) / 2 );

    setLeftVelocity(forward, speed);
    setRightVelocity(forward, speed);
    
    wait(20, msec);
  }

  stopLeft();
  stopRight();

}

// Go forward in whatever direction it was already in
// Trapezoidal motion profiling
void Robot::goForwardUniversal(float distInches, float maxSpeed, float universalAngle, float rampUpInches, float slowDownInches, int timeout, 
  std::function<bool(void)> func) {

  Trapezoid trap(distInches, maxSpeed, FORWARD_MIN_SPEED, rampUpInches, slowDownInches);
  PID anglePID(20, 0, 0.023);
  float turnAngle = universalAngle - gyroSensor.heading(degrees);
  if (turnAngle > 180) turnAngle -= 360;
  else if (turnAngle < -180) turnAngle += 360;

  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();
  gyroSensor.resetRotation();

  // finalDist is 0 if we want driveTimed instead of drive some distance
  while (!trap.isCompleted() && !isTimeout(startTime, timeout)) {

    // if there is a concurrent function to run, run it
    if (func) {
      bool done = func();
      if (done) {
        // if func is done, make it empty
        func = {};
      }
    }

    float error = anglePID.tick(turnAngle - gyroSensor.rotation(), 1);
    float speed = trap.get( (leftMotorA.position(degrees) + rightMotorA.position(degrees)) / 2 );
    float correction = anglePID.tick(error);

    float left = speed + correction * (speed > 0? 1 : -1);
    float right =  speed - correction * (speed > 0? 1 : -1);

    if (fabs(left) > 100) {
      right = right * fabs(100 / left);
      left = fmin(100, fmax(-100, left));
    } else if (fabs(right) > 100) {
      left = left * fabs(100 / right);
      right = fmin(100, fmax(-100, right));
    }

    setLeftVelocity(forward, left);
    setRightVelocity(forward, right);
    
    wait(20, msec);
  }

  stopLeft();
  stopRight();

}

// Trapezoidal motion profiling
// Does not use gyro sensor. 
// distInches is positive if forward, negative if reverse
void Robot::goCurve(float distInches, float maxSpeed, float turnPercent, float rampUpInches, float slowDownInches, int timeout, std::function<bool(void)> func) {

  Trapezoid trap(distInches, maxSpeed, FORWARD_MIN_SPEED, rampUpInches, slowDownInches);


  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();


  // Repeat until either arrived at target or timed out
  while (!trap.isCompleted() && !isTimeout(startTime, timeout)) {

    // if there is a concurrent function to run, run it
    if (func) {
      bool done = func();
      if (done) {
        // if func is done, make it empty
        func = {};
      }
    }

    float speed = trap.get( (leftMotorA.position(degrees) + rightMotorA.position(degrees) / 2) ); 

    setLeftVelocity(forward, speed * (1 + turnPercent));
    setRightVelocity(forward, speed * (1 + turnPercent));

    wait(20, msec);
  }
  logController("done");
  stopLeft();
  stopRight();

}

vision Robot::getCamera(directionType dir, Goal goal) {
  vision camera = vision((dir == forward) ? PORT10 : PORT15, goal.bright, goal.sig);
  // camera->setBrightness(goal.bright);
  // camera->setSignature(goal.sig);
  return camera;
}

void Robot::updateCamera(Goal goal) {
  backCamera = vision(PORT15, goal.bright, goal.sig);
  frontCamera = vision(PORT10, goal.bright, goal.sig);
}

// Go forward with vision tracking towards goal
// PID for distance and for correction towards goal
// distInches is positive if forward, negative if reverse
void Robot::goVision(float distInches, float maxSpeed, Goal goal, directionType cameraDir, float rampUpInches, float slowDownInches,
 int timeout, bool stopAfter, float K_P, std::function<bool(void)> func) {

  updateCamera(goal);

  vision *camera = (cameraDir == forward) ? &frontCamera : &backCamera;
  
  Trapezoid trap(distInches, maxSpeed, FORWARD_MIN_SPEED, rampUpInches, slowDownInches);

  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();

  PID vPID(K_P, 0, 0.2, 0.1, 10, 10);
  logController("start vision");

  while (!trap.isCompleted() && !isTimeout(startTime, timeout)) {

    if (func) {
      if (func()) {
        // if func is done, make it empty
        func = {};
      }
    }

    camera->takeSnapshot(goal.sig);
    
    float correction = 0; // between -1 and 1
    if(camera->largestObject.exists)  correction = vPID.tick((VISION_CENTER_X - camera->largestObject.centerX) / VISION_CENTER_X);

    float speed = trap.get( (leftMotorA.position(degrees) + rightMotorA.position(degrees)) / 2 );

    float left = speed - (fabs(speed) / 50) * correction * (cameraDir == forward? 1 : -1);
    float right =  speed + (fabs(speed) / 50) * correction * (cameraDir == forward? 1 : -1);

    if (fabs(left) > 100) {
      right = right * fabs(100 / left);
      left = fmin(100, fmax(-100, left));
    } else if (fabs(right) > 100) {
      left = left * fabs(100 / right);
      right = fmin(100, fmax(-100, right));
    }

    setLeftVelocity(cameraDir, left);
    setRightVelocity(cameraDir, right);

    wait(20, msec);
  }
  if (stopAfter) {
    stopLeft();
    stopRight();
  }
  logController("stop vison");

}

// Returns true if aligned to goal, false if timed out or maxTurnAngle reached
bool Robot::goTurnVision(Goal goal, bool defaultClockwise, directionType cameraDir, float maxTurnAngle) {


  float delta;
  int timeout = 4; // fix once PID is fine
  int startTime = vex::timer::system();
  updateCamera(goal);

  vision *camera = (cameraDir == forward) ? &frontCamera : &backCamera;
  camera->takeSnapshot(goal.sig);
  if (!camera->largestObject.exists) return false;

  gyroSensor.resetRotation();

  //PID vPID(70, 0, 0.1, 0.1, 3, 20); // took out struct for now because that needs to be fixed
  PID vPID(70, 0, 0, 0.05, 3, 25); // took out struct for now because that needs to be fixed

  while (!vPID.isCompleted()) {

    // failure exit conditions
    // logController("%f", fabs(gyroSensor.rotation()) > maxTurnAngle);
    if (isTimeout(startTime, timeout) || fabs(gyroSensor.rotation()) > maxTurnAngle) return false;

    camera->takeSnapshot(goal.sig);
    
    // correction is between -1 and 1. Positive if overshooting to right, negative if overshooting to left
    if(camera->largestObject.exists)  delta = (VISION_CENTER_X - camera->largestObject.centerX) / VISION_CENTER_X;
    else delta = defaultClockwise ? -1 : 1;

    float speed = vPID.tick(delta, 100);
    setLeftVelocity(reverse, speed);
    setRightVelocity(forward, speed);

    wait(20, msec);

  }

  stopLeft();
  stopRight();

  // did not exit on failure conditions, so successfully aligned
  return true;
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

float Robot::goTurnVision2(Goal goal, directionType cameraDir, float minSpeed, float timeout) {

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

// angleDegrees is positive if clockwise, negative if counterclockwise
void Robot::goTurn(float angleDegrees, std::function<bool(void)> func) {

  PID anglePID(3, 0.00, 0.05, 2, 3, 25);
  //PID anglePID(GTURN_24);

  float timeout = 5;
  float speed;

  log("initing");
  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();
  gyroSensor.resetRotation();
  log("about to loop");

  while (!anglePID.isCompleted() && !isTimeout(startTime, timeout)) {

    if (func) {
      if (func()) {
        // if func is done, make it empty
        func = {};
      }
    }

    speed = anglePID.tick(angleDegrees - gyroSensor.rotation());

    //logController("wtf %f", speed);

    setLeftVelocity(forward, speed);
    setRightVelocity(reverse, speed);

    wait(20, msec);
  }
  logController("wtf done");

  stopLeft();
  stopRight();
}
// Turn to some universal angle based on starting point. Turn direction is determined by smallest angle
void Robot::goTurnU(float universalAngleDegrees, std::function<bool(void)> func) {

  float turnAngle = universalAngleDegrees - gyroSensor.heading(degrees);
  if (turnAngle > 180) turnAngle -= 360;
  else if (turnAngle < -180) turnAngle += 360;

  goTurn(turnAngle, func);
}

// A fast but inaccurate turning function
void Robot::goTurnFast(bool isClockwise, float turnDegrees, float maxSpeed, float minSpeed, float slowDownDegrees, float endSlowDegrees, float timeout, std::function<bool(void)> func) {

  float delta;
  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();
  gyroSensor.resetRotation();

  logController("start turn fast");

  // Repeat until either arrived at target or timed out
  while (fabs(gyroSensor.rotation()) < turnDegrees && !isTimeout(startTime, timeout)) {

    if (func) {
      if (func()) {
        // if func is done, make it empty
        func = {};
      }
    }

    logController("%f", gyroSensor.heading());

    float angle = fabs(gyroSensor.rotation());
    if (turnDegrees - angle < endSlowDegrees) delta = 0;
    else if (turnDegrees - angle < slowDownDegrees + endSlowDegrees) delta = (turnDegrees - endSlowDegrees - angle) / slowDownDegrees;
    else delta = 1;

    float speed = minSpeed + delta * (maxSpeed - minSpeed);

    setLeftVelocity(isClockwise ? forward : reverse, speed);
    setRightVelocity(isClockwise ? reverse : forward, speed);

    wait(20, msec);

  }
  logController("finish turn fast");
  stopLeft();
  stopRight();

}

// Turn to some universal angle based on starting point. Turn direction is determined by smallest angle
void Robot::goTurnFastU(float universalAngleDegrees, float maxSpeed, float minSpeed, float slowDownDegrees, float endSlowDegrees, float timeout, 
std::function<bool(void)> func) {

  float turnAngle = universalAngleDegrees - gyroSensor.heading(degrees);
  if (turnAngle > 180) turnAngle -= 360;
  else if (turnAngle < -180) turnAngle += 360;

  goTurnFast(turnAngle > 0, fabs(turnAngle), maxSpeed, minSpeed, slowDownDegrees, endSlowDegrees, timeout, func);
}

void Robot::openClaw() {
  clawPiston.set(true);
}

void Robot::closeClaw() {
  clawPiston.set(false);
}


void Robot::setLeftVelocity(directionType d, double percent) {
  if (percent < 0) {
    d = (d == forward) ? reverse : forward;
    percent = -percent;
  }
  leftMotorA.spin(d, percent / 100.0 * MAX_VOLTS, voltageUnits::volt);
  leftMotorB.spin(d, percent / 100.0 * MAX_VOLTS, voltageUnits::volt);
  leftMotorC.spin(d, percent / 100.0 * MAX_VOLTS, voltageUnits::volt);
  leftMotorD.spin(d, percent / 100.0 * MAX_VOLTS, voltageUnits::volt);
  leftMotorE.spin(d, percent / 100.0 * MAX_VOLTS, voltageUnits::volt);
}

void Robot::setRightVelocity(directionType d, double percent) {
  if (percent < 0) {
    d = (d == forward) ? reverse : forward;
    percent = -percent;
  }
  rightMotorA.spin(d, percent / 100.0 * MAX_VOLTS, voltageUnits::volt);
  rightMotorB.spin(d, percent / 100.0 * MAX_VOLTS, voltageUnits::volt);
  rightMotorC.spin(d, percent / 100.0 * MAX_VOLTS, voltageUnits::volt);
  rightMotorD.spin(d, percent / 100.0 * MAX_VOLTS, voltageUnits::volt);
  rightMotorE.spin(d, percent / 100.0 * MAX_VOLTS, voltageUnits::volt);
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

void Robot::setBrakeType(brakeType b) {
  leftMotorA.setBrake(b);
  leftMotorB.setBrake(b);
  leftMotorC.setBrake(b);
  leftMotorD.setBrake(b);
  leftMotorE.setBrake(b);

  rightMotorA.setBrake(b);
  rightMotorB.setBrake(b);
  rightMotorC.setBrake(b);
  rightMotorD.setBrake(b);
  rightMotorE.setBrake(b);
}

void Robot::setMaxArmTorque(float c) {
  fourBarLeft.setMaxTorque(c, currentUnits::amp);
  fourBarRight.setMaxTorque(c, currentUnits::amp);
  chainBarLeft.setMaxTorque(c, currentUnits::amp);
  chainBarRight.setMaxTorque(c, currentUnits::amp);
}
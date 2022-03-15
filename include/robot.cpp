#include <math.h>
#include "robot.h"


// Motor ports Left: 1R, 2F, 3F,  20T Right: 12R, 11F, 13F
// gear ratio is 60/36
Robot::Robot(controller* c, bool _isSkills) : leftMotorA(0), leftMotorB(0), leftMotorC(0), leftMotorD(0), leftMotorE(0), rightMotorA(0), rightMotorB(0), 
  rightMotorC(0), rightMotorD(0), rightMotorE(0), fourBarLeft(0), fourBarRight(0), chainBarLeft(0), chainBarRight(0), frontCamera(PORT10), 
  backCamera(PORT15), gyroSensor(PORT4), arm(), buttons(c), gpsSensor(0) {

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

  fourBarLeft = motor(PORT7, ratio36_1, false);
  fourBarRight = motor(PORT14, ratio36_1, true);
  chainBarLeft = motor(PORT9, ratio36_1, false);
  chainBarRight = motor(PORT8, ratio36_1, true);

  gyroSensor = inertial(PORT11);

  arm.init(&buttons, chainBarLeft, chainBarRight, fourBarLeft, fourBarRight, chainBarPot, fourBarBump);

  driveType = TWO_STICK_ARCADE;
  robotController = c; 

  fourBarLeft.setBrake(hold);
  fourBarRight.setBrake(hold);
  chainBarLeft.setBrake(hold);
  chainBarRight.setBrake(hold);

  gpsSensor = gps(PORT2);

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
      printf("{%f, %f},\n",  fourBarRight.position(degrees), chainBarRight.position(degrees));
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
  arm.armMovement(true, 100);

  // if(vex::controller().ButtonY.pressing()) {
  //   // "Disable steppers" except you gotta hold it down
  //   fourBarLeft.setBrake(coast);
  //   fourBarRight.setBrake(coast);
  //   chainBarLeft.setBrake(coast);
  //   chainBarRight.setBrake(coast);
  //   fourBarLeft.stop();
  //   fourBarRight.stop();
  // } else {
  //   fourBarLeft.setBrake(hold);
  //   fourBarRight.setBrake(hold);
  //   chainBarLeft.setBrake(hold);
  //   chainBarRight.setBrake(hold);
  //   arm.armMovement(true, 10);
  // }
  // const float l = fourBarLeft.rotation(vex::degrees), r = fourBarLeft.rotation(vex::degrees);
  // const float ll = chainBarLeft.rotation(vex::degrees), rr = chainBarRight.rotation(vex::degrees);
  // log(6, "Live Vals:");
  // log(7, "FBL %4f   FBR: %4f",l, r);
  // log(8, "CBL %4f   CBR: %4f",ll, rr);

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
  // log("%f %f %f", chainBarLeft.position(deg), (initPot-chainBarPot.value(deg)) * 5, chainBarPot.value(deg));

  buttons.updateButtonState();

}

void Robot::waitGyroCallibrate() {
  if (gyroSensor.isCalibrating()) {
    int i = 0;
    while (gyroSensor.isCalibrating()) {
      wait(20, msec);
      i++;
    }
    gyroSensor.resetRotation();
    wait(1000, msec);
  }
  
  wait(500, msec);
  log("done calibration");
  
}

// return in inches
float Robot::getEncoderDistance() {
  return degreesToDistance((leftMotorA.rotation(deg) + rightMotorA.rotation(deg)) / 2);
}

void Robot::resetEncoderDistance() {
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();
}

float Robot::getAngle() {
  return gyroSensor.heading();
}

// If the GPS is known to be reliable at a location, and the gyro heading is close enough to gps heading, recalibrate gyro heading
// (assuming gps sensor is on left side of robot at the moment)
void Robot::possiblyResetGyroWithGPS() {
  
  if (gpsSensor.quality() != 100) return; // must be gps localized at current frame

  float gpsAngle = gpsSensor.heading() + 180;
  if (fabs(getAngle() - gpsAngle) < 10) {
    logController("YES set heading\nfrom:%f\nto:%f", getAngle(), gpsAngle);
    gyroSensor.setHeading(gpsAngle, degrees);
  } else {
    logController("NO set heading\nfrom:%f\nto:%f", getAngle(), gpsAngle);
  }

}

void Robot::goForwardTimed(float duration, float speed) {

  int startTime = vex::timer::system();

  while (!isTimeout(startTime, duration)) {
    setLeftVelocity(forward, speed);
    setRightVelocity(forward, speed);
    wait(20, msec);
  }
  stopLeft();
  stopRight();

}

// Go forward a number of inches, maintaining a specific heading if angleCorrection = true
void Robot::goForwardU(float distInches, float maxSpeed, float universalAngle, float slowDownInches, float minSpeed,
bool stopAfter, std::function<bool(void)> func, float timeout) {

  resetEncoderDistance();

  Trapezoid trap(0, distInches, maxSpeed, minSpeed, slowDownInches, 0);
  PID turnPID(1, 0.00, 0);

  float currDist;
  int startTime = vex::timer::system();
  

  //log("start forward %f %f %f", distInches, startX, startY);

  while (!trap.isCompleted() && !isTimeout(startTime, timeout)) {

    // if there is a concurrent function to run, run it
    if (func) {
      bool done = func();
      if (done) {
        // if func is done, make it empty
        func = {};
      }
    }

    currDist = getEncoderDistance();
    
    float speed = trap.tick(currDist);
    float ang = getAngleDiff(universalAngle, getAngle());
    float correction = turnPID.tick(ang);
 
    setLeftVelocity(forward, speed + correction);
    setRightVelocity(forward, speed - correction);

    //log("Target: %f\nActual:%f\nLeft:%f\nRight:%f\n", universalAngle, getAngle(), speed+correction, speed-correction);
    log("Heading: %f", gyroSensor.heading());

    wait(20, msec);
  }
  if (stopAfter) {
    stopLeft();
    stopRight();
  }
  log("straight done");
}

// Go forward with standard internal encoder wheels for distance, and no angle correction
void Robot::goForward(float distInches, float maxSpeed, float slowDownInches, float minSpeed,
bool stopAfter, std::function<bool(void)> func, float timeout) {
  goForwardU(distInches, maxSpeed, getAngle(), slowDownInches, minSpeed, stopAfter, func, timeout);
}

void Robot::goCurve(float distInches, float maxSpeed, float turnPercent, float slowDownInches, float minSpeed, 
bool stopAfter, std::function<bool(void)> func) {

  float timeout = 5;
  resetEncoderDistance();

  Trapezoid trap(0, distInches, maxSpeed, minSpeed, slowDownInches, 0);

  int startTime = vex::timer::system();

  while (!trap.isCompleted() && !isTimeout(startTime, timeout)) {

     // if there is a concurrent function to run, run it
    if (func) {
      bool done = func();
      if (done) {
        // if func is done, make it empty
        func = {};
      }
    }
  
    float speed = trap.tick(getEncoderDistance());

    // turnPercent bounded between -1 (counterclockwise point turn) and 1 (clockwise point turn)
    float lspeed, rspeed;
    if (turnPercent >= 0) {
      lspeed = 1;
      rspeed = 1 - 2*turnPercent;
    } else {
      rspeed = 1;
      lspeed = 1 + 2*turnPercent;
    }

    setLeftVelocity(forward, lspeed * speed);
    setRightVelocity(forward, rspeed * speed);

    wait(20, msec);
  }

  if (stopAfter) {
    stopLeft();
    stopRight();
  }
}

// Turn to some universal angle based on starting point. Turn direction is determined by smallest angle to universal angle
void Robot::goTurnU_PID(float universalAngleDegrees, bool stopAfter, float timeout) {

  PID anglePID(2, 0, 0.13, 1.5, 5, 25, 75);

  float speed;

  log("initing");
  int startTime = vex::timer::system();
  log("about to loop");

  while (!anglePID.isCompleted() && !isTimeout(startTime, timeout)) {

    float ang = getAngleDiff(universalAngleDegrees, getAngle());
    speed = anglePID.tick(ang);

    //log("Turn \nTarget: %f \nCurrent: %f \nDiff: %f\nSpeed: %f \nGPS: %f", universalAngleDegrees, getAngle(), ang, speed, GPS11.heading());
    //log("heading: %f", GPS11.heading());
    log("%f", gyroSensor.heading());

    setLeftVelocity(forward, speed);
    setRightVelocity(reverse, speed);

    wait(20, msec);
  }
  //log("wtf done");

  if (stopAfter) {
    stopLeft();
    stopRight();
  }  
}

// the same thing as goTurnU_PID but using trapezoidal
void Robot::goTurnU_TRAP(float universalAngleDegrees, bool stopAfter, float timeout) {

  const float MAX_SPEED = 100;
  const float MIN_SPEED = 30;

  float initAngle = getAngleDiff(universalAngleDegrees, getAngle()); // don't just use getAngle() because wraparound
  Trapezoid angleTrap(initAngle, 0, MAX_SPEED, MIN_SPEED, 50, 25);

  log("initing");
  int startTime = vex::timer::system();
  log("about to loop");

  while (!angleTrap.isCompleted() && !isTimeout(startTime, timeout)) {

    float ang = getAngleDiff(universalAngleDegrees, getAngle());
    float speed = angleTrap.tick(ang);

    //log("Turn \nTarget: %f \nCurrent: %f \nDiff: %f\nSpeed: %f \nGPS: %f", universalAngleDegrees, getAngle(), ang, speed, GPS11.heading());
    //log("heading: %f", GPS11.heading());
    log("Heading: %f\nCurrentAngle: %f", gyroSensor.heading(), ang);

    setLeftVelocity(forward, speed);
    setRightVelocity(reverse, speed);

    wait(20, msec);
  }
  //log("wtf done");

  if (stopAfter) {
    stopLeft();
    stopRight();
  }  
}

void Robot::updateCamera(Goal goal) {
  backCamera = vision(PORT15, goal.bright, goal.sig);
  frontCamera = vision(PORT10, goal.bright, goal.sig);
}



// Go forward with vision tracking towards goal
// PID for distance and for correction towards goal
// distInches is positive if forward, negative if reverse
void Robot::goVision(float distInches, float speed, Goal goal, float slowDownInches, float minSpeed, bool stopAfter, float timeout) {

  resetEncoderDistance();

  Trapezoid trapDist(0, distInches, speed, minSpeed, slowDownInches);
  PID pidTurn(25, 0, 0);

  updateCamera(goal);

  int startTime = vex::timer::system();
  vision *camera = (distInches > 0) ? &frontCamera : &backCamera;

  // forward until the maximum distance is hit, the timeout is reached, or limitSwitch is turned on
  while (!trapDist.isCompleted() && !isTimeout(startTime, timeout)) {

    camera->takeSnapshot(goal.sig);
    
    float correction = camera->largestObject.exists ? pidTurn.tick((VISION_CENTER_X-camera->largestObject.centerX) / VISION_CENTER_X) : 0;
    float speed = trapDist.tick(getEncoderDistance());

    setLeftVelocity(forward, speed - correction);
    setRightVelocity(forward, speed + correction);

    wait(20, msec);
  }
  if (stopAfter) {
    stopLeft();
    stopRight();
  }
}

// Align to the goal of specified color with PID
void Robot::goAlignVision(Goal goal, directionType cameraDir, float timeout) {

  updateCamera(goal);

  int startTime = vex::timer::system();
  float speed = 0;

  PID vTurnPID(40, 0, 1, 0.05, 3, 12);
  vision *camera = (cameraDir == forward) ? &frontCamera : &backCamera;

  while (!vTurnPID.isCompleted() && !isTimeout(startTime, timeout)) {

    camera->takeSnapshot(goal.sig);

    if (camera->largestObject.exists) speed = vTurnPID.tick((VISION_CENTER_X-camera->largestObject.centerX) / VISION_CENTER_X);

    setLeftVelocity(reverse, speed);
    setRightVelocity(forward, speed);

    wait(20, msec);
  }

  stopLeft();
  stopRight();
}


void Robot::driveArmDown(float duration) {
  fourBarLeft.spin(reverse, 10, percent);
  fourBarRight.spin(reverse, 10, percent);
  int startTime = vex::timer::system();
  while (!isTimeout(startTime, duration)) {
    wait(20, msec);
  }
  fourBarLeft.stop(hold);
  fourBarRight.stop(hold);

  // relieve pressure on ground a bit
  fourBarLeft.startRotateFor(forward, 10, degrees);
  fourBarLeft.startRotateFor(forward, 10, degrees);
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

void Robot::setMaxDriveTorque(float c) {
  leftMotorA.setMaxTorque(c, currentUnits::amp);
  leftMotorB.setMaxTorque(c, currentUnits::amp);
  leftMotorC.setMaxTorque(c, currentUnits::amp);
  leftMotorD.setMaxTorque(c, currentUnits::amp);
  leftMotorE.setMaxTorque(c, currentUnits::amp);

  rightMotorA.setMaxTorque(c, currentUnits::amp);
  rightMotorB.setMaxTorque(c, currentUnits::amp);
  rightMotorC.setMaxTorque(c, currentUnits::amp);
  rightMotorD.setMaxTorque(c, currentUnits::amp);
  rightMotorE.setMaxTorque(c, currentUnits::amp);
}
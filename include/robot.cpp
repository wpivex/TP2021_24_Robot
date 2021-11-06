#include "robot.h"
#include <math.h>

// Motor ports Left: 1R, 2F, 3F,  20T Right: 12R, 11F, 13F
// gear ratio is 60/36
Robot::Robot(controller* c) : leftMotorA(0), leftMotorB(0), leftMotorC(0), leftMotorD(0), leftMotorE(0), rightMotorA(0), rightMotorB(0), 
  rightMotorC(0), rightMotorD(0), rightMotorE(0), fourBarLeft(0), fourBarRight(0), chainBarLeft(0), chainBarRight(0), claw(0),
  fourBarFake(0), chainBarFake(0), camera(0) {
  leftMotorA = motor(PORT1, ratio18_1, true);
  leftMotorB = motor(PORT2, ratio18_1, true);
  leftMotorC = motor(PORT3, ratio18_1, true);
  leftMotorD = motor(PORT4, ratio18_1, true);
  leftMotorE = motor(PORT5, ratio18_1, true);
  leftDrive = motor_group(leftMotorA, leftMotorB, leftMotorC, leftMotorD, leftMotorE);

  rightMotorA = motor(PORT13, ratio18_1, false);
  rightMotorB = motor(PORT14, ratio18_1, false);
  rightMotorC = motor(PORT15, ratio18_1, false);
  rightMotorD = motor(PORT16, ratio18_1, false);
  rightMotorE = motor(PORT17, ratio18_1, false);
  rightDrive = motor_group(rightMotorA, rightMotorB, rightMotorC, rightMotorD, rightMotorE);

  leftMotorA.setMaxTorque(5, torqueUnits::Nm);
  leftMotorB.setMaxTorque(5, torqueUnits::Nm);
  leftMotorC.setMaxTorque(5, torqueUnits::Nm);
  leftMotorD.setMaxTorque(5, torqueUnits::Nm);
  leftMotorE.setMaxTorque(5, torqueUnits::Nm);

  rightMotorA.setMaxTorque(5, torqueUnits::Nm);
  rightMotorB.setMaxTorque(5, torqueUnits::Nm);
  rightMotorC.setMaxTorque(5, torqueUnits::Nm);
  rightMotorD.setMaxTorque(5, torqueUnits::Nm);
  rightMotorE.setMaxTorque(5, torqueUnits::Nm);

  fourBarLeft = motor(10, ratio18_1, true);
  fourBarRight = motor(8, ratio18_1, false);
  chainBarLeft = motor(9, ratio18_1, true);
  chainBarRight = motor(19, ratio18_1, false);
  claw = motor(16, ratio18_1, true);

  driveType = TANK;
  robotController = c; 

  fourBarFake = motor(PORT19, ratio18_1, false);
  chainBarFake = motor(PORT12, ratio18_1, true);

  vision::signature SIG_1 (1, 1999, 2599, 2299, -3267, -2737, -3002, 2.500, 0);
  camera = vision(PORT6, 50, SIG_1);

  fourBarLeft.setBrake(hold);
  fourBarRight.setBrake(hold);
  chainBarLeft.setBrake(hold);
  chainBarRight.setBrake(hold);
  claw.setBrake(hold);
}

brain Brain;
controller Controller1;

void Robot::teleop() {
  float leftJoystick = (driveType == ARCADE) ? robotController->Axis3.position()^3 + robotController->Axis1.position()^3: robotController->Axis3.position()^3;
  float rightJoystick = (driveType == ARCADE) ? robotController->Axis3.position()^3 + robotController->Axis1.position()^3: robotController->Axis2.position()^3;

  Controller1.Screen.clearScreen();

  float oldLeft = 0;
  float oldRight = 0;

  if (fabs(leftJoystick) > 5) {
    float percent = (driveType == ARCADE) ? (pow((robotController->Axis3.position()/100.00f), 3.00f) + pow((robotController->Axis1.position()/100.00f), 5.00f))*100.00f : leftJoystick;
    setLeftVelocity(forward, percent/fabs(percent)*fmin(fabs(percent), 100)); //60 is max for now
    oldLeft = percent;
  } else {
    stopLeft();
  }

  if (fabs(rightJoystick) > 5) {
    float percent = (driveType == ARCADE) ? (pow((robotController->Axis3.position()/100.00f), 3.00f) - pow((robotController->Axis1.position()/100.00f), 5.00f))*100.00f : rightJoystick;
    setRightVelocity(forward, percent/fabs(percent)*fmin(fabs(percent), 100));
    oldRight = percent;
  } else {
    stopRight();
  }
  Controller1.Screen.setCursor(0, 0);
  float maxTorqueLeft = fmax(fmax(fmax(fmax(leftMotorA.torque(), leftMotorB.torque()), leftMotorC.torque()), leftMotorD.torque()), leftMotorE.torque());
  float maxTorqueRight = fmax(fmax(fmax(fmax(rightMotorA.torque(), rightMotorB.torque()), rightMotorC.torque()), rightMotorD.torque()), rightMotorE.torque());
  Controller1.Screen.print("%f, %f", leftMotorA.torque(torqueUnits::Nm), rightMotorA.efficiency());
  wait(100, msec);
}

float distanceToDegrees(float dist) {
  return dist * 360 / 60 * 36 / 2 / M_PI / (3.25 / 2);
}

void Robot::driveStraight(float percent, float dist) {
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();
  // currPos is the current average encoder position, travelDist is the total encoder distance to be traversed, 
  // targetDist is the target encoder position, and currLeft/Right are the current left and right encoder positions
  float currLeft = leftMotorA.position(degrees);
  float currRight = rightMotorA.position(degrees);
  float currPos = (currLeft + currRight) / 2;
  float travelDist = distanceToDegrees(dist);
  float targetDist = currPos + travelDist;
  
  while (currPos < targetDist) {
    setLeftVelocity(dist > 0 ? forward : reverse, 5 + (percent - 5) * ((targetDist - currPos) / travelDist));
    setRightVelocity(dist > 0 ? forward : reverse, 5 + (percent - 5) * ((targetDist - currPos) / travelDist) - ((currRight - currLeft) / travelDist * 10));
    currLeft = leftMotorA.position(degrees);
    currRight = rightMotorA.position(degrees);
    currPos = (currLeft + currRight) / 2;
  }
  leftDrive.stop();
  rightDrive.stop();
}

void Robot::driveTimed(float percent, float driveTime) {
  int milliseconds = vex::timer::system();
  while (vex::timer::system() < milliseconds + driveTime) {
    setLeftVelocity(forward, percent);
    setRightVelocity(forward, percent);
  }
  leftDrive.stop();;
  rightDrive.stop();;
}

//12.375 in wheelbase
void Robot::turnToAngle(float percent, float turnAngle) {
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();
  // currPos is the current average encoder position, travelDist is the total encoder distance to be traversed, 
  // and targetDist is the target encoder position
  float currPos = (fabs(leftMotorA.position(degrees)) + fabs(rightMotorA.position(degrees))) / 2;
  float travelDist = distanceToDegrees(turnAngle / 360 * 2 * M_PI * (12.375 / 2));
  float targetDist = currPos + travelDist;
  
  while (currPos < targetDist) {
    setLeftVelocity(turnAngle > 0 ? forward : reverse, 5 + (percent - 5) * ((targetDist - currPos) / travelDist));
    setRightVelocity(turnAngle > 0 ? reverse : forward, 5 + (percent - 5) * ((targetDist - currPos) / travelDist));
    currPos = (fabs(leftMotorA.position(degrees)) + fabs(rightMotorA.position(degrees))) / 2;
  }
  leftDrive.stop();;
  rightDrive.stop();;
}

void Robot::openClaw() {}
void Robot::closeClaw() {}
void Robot::liftFourBar(float percentHeight) {}
void Robot::lowerFourBar(float percentHeight) {}

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
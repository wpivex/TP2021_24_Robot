#include "robot.h"
#include <math.h>

// Motor ports Left: 1R, 2F, 3F,  20T Right: 12R, 11F, 13F
// gear ratio is 60/36
Robot::Robot(controller* c) : leftMotorA(0), leftMotorB(0), leftMotorC(0), /*leftMotorD(0), leftMotorE(0),*/ rightMotorA(0), rightMotorB(0), 
  rightMotorC(0), /*rightMotorD(0), rightMotorE(0),*/ fourBarLeft(0), fourBarRight(0), chainBarLeft(0), chainBarRight(0), claw(0),
  fourBarFake(0), chainBarFake(0) {
  leftMotorA = motor(PORT1, ratio18_1, false);
  leftMotorB = motor(PORT2, ratio18_1, true);
  leftMotorC = motor(PORT3, ratio18_1, true);
  // leftMotorD = motor(0, ratio18_1, true);
  // leftMotorE = motor(0, ratio18_1, true);
  leftDrive = motor_group(leftMotorA, leftMotorB, leftMotorC/*, leftMotorD, leftMotorE*/);

  rightMotorA = motor(PORT12, ratio18_1, false);
  rightMotorB = motor(PORT11, ratio18_1, false);
  rightMotorC = motor(PORT13, ratio18_1, false);
  // rightMotorD = motor(0, ratio18_1, true);
  // rightMotorE = motor(0, ratio18_1, false);
  rightDrive = motor_group(rightMotorA, rightMotorB, rightMotorC/*, rightMotorD, rightMotorE*/);

  // fourBarLeft = motor(10, ratio18_1, true);
  // fourBarRight = motor(20, ratio18_1, false);
  // chainBarLeft = motor(9, ratio18_1, true);
  // chainBarRight = motor(19, ratio18_1, false);
  // claw = motor(16, ratio18_1, true);

  driveType = ARCADE;
  robotController = c; 

  fourBarFake = motor(PORT20, ratio18_1, false);
  chainBarFake = motor(PORT12, ratio18_1, true);
}

void Robot::teleop() {
  robotController->Screen.clearScreen();
  if(robotController->ButtonA.pressing()) {
    robotController->Screen.setCursor(0, 0);
    robotController->Screen.print("FOURBAR");
    fourBarFake.rotateTo(45, degrees, false);
  }
  // robotController->Screen.clearScreen();

  // float leftJoystick = (driveType == ARCADE) ? robotController->Axis3.position()^3 + robotController->Axis1.position()^3: robotController->Axis3.position()^3;
  // float rightJoystick = (driveType == ARCADE) ? robotController->Axis3.position()^3 + robotController->Axis1.position()^3: robotController->Axis2.position()^3;

  // if (fabs(leftJoystick) > 5) {
  //   float percent = (driveType == ARCADE) ? (pow((robotController->Axis3.position()/100.00f), 3.00f) + pow((robotController->Axis1.position()/100.00f), 5.00f))*100.00f : leftJoystick;
  //   setLeftVelocity(forward, percent);
  // } else {
  //   leftDrive.stop();;
  // }

  // if (fabs(rightJoystick) > 5) {
  //   float percent = (driveType == ARCADE) ? (pow((robotController->Axis3.position()/100.00f), 3.00f) - pow((robotController->Axis1.position()/100.00f), 5.00f))*100.00f : rightJoystick;
  //   setRightVelocity(forward, percent);
  // } else {
  //   rightDrive.stop();;
  // }
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
  leftDrive.stop();;
  rightDrive.stop();;
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
  leftDrive.spin(d, percent, percentUnits::pct);
}

void Robot::setRightVelocity(directionType d, double percent) {
  rightDrive.spin(d, percent, percentUnits::pct);
}
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Vision6              vision        6               
// DigitalInE           digital_in    E               
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "../include/robot.cpp"

const bool IS_SKILLS = true;

competition Competition;
controller Controller1(controllerType::primary);

Robot mainBot = Robot(&Controller1, IS_SKILLS);

int mainTeleop() {
  mainBot.setBackClamp(false);
  mainBot.setFrontClamp(false);
  while (true) {
    mainBot.teleop();
    wait(20, msec);
  }
  return 0;
}

void userControl(void) { task controlLoop1(mainTeleop); }

void middleFirst(void) {
  int color = 1; //red is 1, blue is 2
  mainBot.setBackClamp(true);
  mainBot.driveCurved(reverse, 20, 55);
  wait(2000, msec);
  mainBot.goForwardVision(true, 100, 30, 25, 0);
  mainBot.driveStraight(100, -10);
  mainBot.setBackClamp(false);
  
  //Go back 2 ft
  mainBot.driveStraight(100, 24);
  mainBot.blindAndVisionTurn(80, 0);
  
  mainBot.setFrontClamp(true);
  mainBot.goForwardVision(false, 100, 24, 40, 0);
  mainBot.setFrontClamp(false);
  wait(250, msec);

  //Go back 2 ft
  mainBot.driveStraight(100, -15);
  mainBot.intakeOverGoal(color);
}

void sideFirst() {
  int color = 1;
  mainBot.setBackClamp(true);
  mainBot.goForwardVision(true, 100, 48, 25, 0);
  mainBot.stopLeft();
  mainBot.stopRight();
  mainBot.setBackClamp(false);
  mainBot.driveStraight(100, 30);
  mainBot.turnToAngle(100, 90, true, reverse);
  mainBot.moveArmToPosition(mainBot.ABOVE_MIDDLE, 100);
  mainBot.turnAndAlignVision(true, color, 0.1, true);
  mainBot.moveArmToPosition(mainBot.INTAKING, 100);
  mainBot.openClaw();
  mainBot.driveStraight(20, 13);
  mainBot.closeClaw();
  mainBot.moveArmToPosition(mainBot.PLACE_GOAL, 100);
  mainBot.openClaw();
  mainBot.moveArmToPosition(mainBot.ABOVE_MIDDLE, 100);
}

void skills() {
  // Get center goal
  mainBot.openClaw();
  mainBot.moveArmToPosition(mainBot.ABOVE_MIDDLE, 100);
  mainBot.driveCurved(forward, 20, -54);
  mainBot.goForwardVision(false, 30, 18, 15, 0);
  mainBot.moveArmToPosition(mainBot.INTAKING, 100);
  mainBot.driveStraight(20, 7);
  mainBot.closeClaw();

  // Place center goal
  mainBot.moveArmToPosition(mainBot.ABOVE_MIDDLE, 100);
  mainBot.turnToAngle(100, 125, true, reverse);
  mainBot.driveStraight(60, 20);
  mainBot.moveArmToPosition(mainBot.PLATFORM_LEVEL, 70);
  mainBot.openClaw();

  // Get right goal
  mainBot.driveStraight(100, -5);
  mainBot.turnToAngle(100, 160, true, reverse);
  mainBot.turnAndAlignVision(false, 0, 0.05, false);
  mainBot.openClaw();
  mainBot.moveArmToPosition(mainBot.INTAKING, 100);
  mainBot.goForwardVision(false, 20, 16, 15, 0);
  mainBot.closeClaw();
  mainBot.moveArmToPosition(mainBot.ABOVE_MIDDLE, 100);

  // Place right goal
  mainBot.turnToAngle(100, -150, true, forward);
  mainBot.driveStraight(20, 24);
  mainBot.moveArmToPosition(mainBot.PLATFORM_LEVEL, 65);
  mainBot.openClaw();

  // Get red goal
  mainBot.driveStraight(100, -10);
  mainBot.turnToAngle(100, -110, true, forward);
  mainBot.turnAndAlignVision(true, 1, 0.1, false);
  mainBot.openClaw();
  mainBot.goForwardVision(false, 100, 60, 25, 1);
  mainBot.moveArmToPosition(mainBot.INTAKING, 100);
  mainBot.driveStraight(20, 5);
  mainBot.closeClaw();
  mainBot.moveArmToPosition(mainBot.ABOVE_MIDDLE, 100);

  // Place red goal
  mainBot.turnToAngle(100, 180, true, forward);
  mainBot.driveStraight(100, 35);
  mainBot.turnToAngle(100, 45, true, forward);
  mainBot.driveStraight(25, 40);
  mainBot.moveArmToPosition(mainBot.PLATFORM_LEVEL, 65);
  mainBot.openClaw();

  // Get blue goal
  mainBot.driveStraight(100, -5);
  mainBot.turnToAngle(100, 90, true, reverse);
  mainBot.turnAndAlignVision(true, 2, 0.1, false);
  mainBot.goForwardVision(false, 100, 20, 25, 2);
  mainBot.moveArmToPosition(mainBot.INTAKING, 100);
  mainBot.driveStraight(20, 10);
  mainBot.closeClaw();

  // Run with blue goal
  mainBot.moveArmToPosition(mainBot.PLACE_GOAL, 100);
  mainBot.turnToAngle(100, 90, true, reverse);
  mainBot.driveStraight(100, 30);
}

int tetherAuto(void) { return 0; }

void autonomous() { thread auto1(skills); }

void testArmValues() {
  mainBot.fourBarLeft.setBrake(coast);
  mainBot.fourBarRight.setBrake(coast);
  mainBot.chainBarLeft.setBrake(coast);
  mainBot.chainBarRight.setBrake(coast);

  while (true) {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("%f", mainBot.fourBarRight.position(degrees));
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("%f", mainBot.chainBarRight.position(degrees));
    wait(20, msec);
  }
}

int main() {
  mainBot.setBackClamp(false);
  mainBot.setFrontClamp(false);

  // Reset location of arm
  mainBot.initArmAndClaw();

  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);
  //testArmValues();

  while (true) {
    wait(20, msec);
  }
}

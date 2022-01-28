// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Vision6              vision        6               
// DigitalInE           digital_in    E               
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "../include/robot.cpp"

competition Competition;
controller Controller1(controllerType::primary);

Robot mainBot = Robot(&Controller1);

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

void mainAuto(void) {
  int color = 1; //red is 1, blue is 2
  mainBot.setBackClamp(true);
  mainBot.driveCurved(reverse, 20, 55);
  mainBot.goForwardVision(true, 100, 40, 30, 0);
  mainBot.setBackClamp(false);
  
  wait(1000, msec);
  //go back 2 ft
  mainBot.driveStraight(100, 24);
  mainBot.blindAndVisionTurn(60, 0);

  wait(1000, msec);
  
  mainBot.setFrontClamp(true);
  mainBot.goForwardVision(false, 100, 30, 40, 0);
  mainBot.setFrontClamp(false);

  wait(1000, msec);
  //Go back 2 ft
  mainBot.driveStraight(100, -24);

  wait(1000, msec);

  mainBot.doCursedAutonStuff(color);
}

void skills() {
  int color = 1;
  mainBot.openClaw();
  mainBot.driveCurved(forward, 20, 55);
  mainBot.goForwardVision(false, 100, 40, 25, 0);
  mainBot.closeClaw();
  mainBot.turnToAngle(100, 110, true, forward);
  mainBot.driveStraight(100, 30);
  // put arm up
  mainBot.driveStraight(100, 10);
  mainBot.openClaw();
  mainBot.driveStraight(100, -10);
  mainBot.turnAndAlignVision(false, 0, 40);
  mainBot.openClaw();
  mainBot.goForwardVision(true, 100, 40, 25, 0);
  mainBot.closeClaw();
  mainBot.turnToAngle(100, 160, true, forward);
  // put arm up
  mainBot.driveCurved(forward, 40, 15);
  mainBot.openClaw();
  mainBot.driveStraight(100, 10);
  mainBot.turnToAngle(100, 110, true, forward);
  mainBot.turnAndAlignVision(true, color, 40);
  mainBot.openClaw();
  mainBot.goForwardVision(false, 100, 100, 25, color);
  mainBot.closeClaw();
  mainBot.turnToAngle(100, 180, true, forward);
  // put arm up
  mainBot.driveStraight(100, 100);
  mainBot.openClaw();
  mainBot.driveStraight(100, -10);
}

int tetherAuto(void) { return 0; }

void autonomous() { thread auto1(mainAuto); }

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
  // testArmValues();

  while (true) {
    wait(20, msec);
  }
}

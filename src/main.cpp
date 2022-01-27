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
    wait(100, msec);
  }
  return 0;
}

void userControl(void) { task controlLoop1(mainTeleop); }

void mainAuto(void) {
  int color = 1; //red is 1, blue is 2
  mainBot.setBackClamp(true);
  mainBot.driveCurved(reverse, 20, 55);
  mainBot.goForwardVision(true, 100, 40, 40, 0);
  mainBot.setBackClamp(false);
  
  mainBot.blindAndVisionTurn(40, 0);

  mainBot.setFrontClamp(true);
  mainBot.goForwardVision(false, 100, 30, 40, 0);
  mainBot.setFrontClamp(false);

  // wait(2000, msec);

  mainBot.turnToAngle(100, -40, false, forward);

  // wait(2000, msec);

  mainBot.turnAndAlignVision(true, color);

  // wait(2000, msec);

  mainBot.goForwardVision(false, 20, 40, 40, color);
  mainBot.moveArmToPosition(0, 50);
  mainBot.openClaw();
  mainBot.driveStraight(20, 12);
  mainBot.closeClaw();
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
    wait(100, msec);
  }
}

int main() {
  mainBot.setBackClamp(false);
  mainBot.setFrontClamp(false);

  // Reset location of arm
  mainBot.initArmAndClaw();

  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);

  while (true) {
    wait(100, msec);
  }
}

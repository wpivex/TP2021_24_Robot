// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Vision6              vision        6               
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "../include/robot.cpp"

competition Competition;
controller Controller1(controllerType::primary);

Robot mainBot = Robot(&Controller1);

int mainTeleop() {
  while (true) {
    mainBot.teleop();
    wait(100, msec);
  }
  return 0;
}

void userControl(void) { task controlLoop1(mainTeleop); }

void mainAuto(void) {
  mainBot.goForwardVision(true, 20, -1);
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
    // Brain.Screen.setCursor(2, 1);
    // Brain.Screen.print((int) mainBot.chainBarRight.position(degrees));
    wait(100, msec);
  }
}

int main() {
  // Reset location of arm
  mainBot.initArm();

  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);

  while (true) {
    wait(100, msec);
  }
}


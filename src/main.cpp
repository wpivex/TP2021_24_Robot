// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Vision6              vision        6               
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "../include/robot.cpp"

competition Competition;
controller Controller1(controllerType::primary);
brain Brain;

Robot mainBot = nullptr;

int mainTeleop() {
  while (true) {
    mainBot.teleop();
    wait(100, msec);
  }
  return 0;
}

void userControl(void) { task controlLoop1(mainTeleop); }

void mainAuto(void) {

  Brain.Screen.render(true,false);
  Brain.Screen.clearLine(0,color::black);
  Brain.Screen.clearLine(1,color::black);
  Brain.Screen.clearLine(2,color::black);
  Brain.Screen.clearLine(3,color::black);
  Brain.Screen.clearLine(4,color::black);
  Brain.Screen.clearLine(6,color::black);
  Brain.Screen.clearLine(8,color::black);
  Brain.Screen.setCursor(1,1);
  Brain.Screen.print("Test");
  Brain.Screen.render();
  
  mainBot.driveCurved(reverse, 34, 45);
  // //wait(1000,msec);
  mainBot.goForwardVision(25);
  mainBot.turnToAngle(100, -30, false);
  mainBot.turnAndAlignVision(false);
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
  mainBot = Robot(&Controller1); 

  // Reset location of arm
  mainBot.initArm();

  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);

  // testArmValues();

  while (true) {
    // Brain.Screen.clearScreen();
    // Brain.Screen.setCursor(1, 1);
    // Brain.Screen.print("%d %d", (int)mainBot.chainBarLeft.position(degrees), (int)mainBot.fourBarLeft.position(degrees));
    wait(100, msec);
  }
}


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
  mainBot.setBackClamp(true);
  mainBot.driveCurved(reverse, 20, 57);
  mainBot.goForwardVision(true, 100, 40, 40);
  mainBot.setBackClamp(false);
  
  // Concurrently raise arm and turn robot (first blind then vision) concurrently, so use non-blocking method calls
  bool armFinished = true;
  bool blindTurnFinished = false;

  mainBot.setArmDestination(2);
  int targetDist = mainBot.getTurnAngle(65);

  while (true) {
    armFinished = mainBot.armMovement(false, 100);
    if (!blindTurnFinished) {
      blindTurnFinished = mainBot.turnToAngleNonblocking(100, targetDist, false, reverse);
    } else {
      mainBot.stopLeft();
      mainBot.stopRight();
    }
   
    if (armFinished && blindTurnFinished) break;
    wait(100,msec);
  }

  mainBot.setFrontClamp(true);
  mainBot.goForwardVision(false, 100, 40, 40);
  mainBot.setFrontClamp(false);
  // mainBot.turnToAngle(100, 90, false, forward);
  // mainBot.turnAndAlignVision(true);
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
  mainBot.setBackClamp(false);
  mainBot.setFrontClamp(false);

  // Reset location of arm
  mainBot.initArm();

  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);

  while (true) {
    wait(100, msec);
  }
}

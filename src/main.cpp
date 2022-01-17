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
  while (true) {
    mainBot.teleop();
    wait(100, msec);
  }
  return 0;
}

void userControl(void) { task controlLoop1(mainTeleop); }

void mainAuto(void) {
  mainBot.driveCurved(reverse, 25, 43);
  mainBot.goForwardVision(true, 20);
  
  // Concurrently raise arm and turn robot (first blind then vision) concurrently, so use non-blocking method calls
  bool armFinished = true;
  bool turnFinished = false;
  bool blindTurnFinished = false;

  mainBot.setArmDestination(2);
  int targetDist = mainBot.getTurnAngle(30);

  int i = 0;
  while (true) {
    //armFinished = mainBot.armMovement(false, 100);
    if (!blindTurnFinished) {
      blindTurnFinished = mainBot.turnToAngleNonblocking(100, targetDist, false, reverse);
      i++;
    } else {
        //turnFinished = mainBot.turnAndAlignVisionNonblocking(false);
        turnFinished = true;
    }
    if (armFinished && turnFinished) break;
    wait(100,msec);
  }

  mainBot.stopLeft();
  mainBot.stopRight();
  wait(1000, msec);

  mainBot.goForwardVision(false, 20);
  mainBot.turnToAngle(100, 90, false, forward); //really turn with vision
  mainBot.turnAndAlignVision(true);
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
  //mainBot.initArm();

  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);

  while (true) {
    wait(100, msec);
  }
}

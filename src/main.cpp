// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Vision6              vision        6               
// DigitalInE           digital_in    E               
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "../include/robot.cpp"

competition Competition;

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

/*
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
*/

int matchAuto() {
  Goal allianceColor = RED;
  mainBot.setBrakeType(hold);

  // ~~~~~~~~~~~~~ Box Rush Right ~~~~~~~~~~~~~~~
  // mainBot.setArmPercent(reverse, 65);
  mainBot.setArmDegrees(25);

  mainBot.openClaw();
  // Drive forwards at full speed (while adjusting towards goal if needed)
  mainBot.goForward(34, 100, 3, 20); 
  mainBot.closeClaw();
  wait(200, msec);
  // Raise arm a bit (so that other team cannot grab it)
  // mainBot.setArmPercent(forward, 50);
  mainBot.setArmDegrees(215);
  // Maybe use concurrency to raise arm while reversing
  // Maybe another driveForward variety with a method that triggers after a certain distance
  // RETREAT
  mainBot.goForward(-24, 100); 
  
  // mainBot.setArmPercent(forward, 0);

  // // ~~~~~~~~~~~ Middle Goal Check ~~~~~~~~~~~~~~
  mainBot.encoderTurnU(120);
  mainBot.setBackClamp(true);
  mainBot.goVision(-24*sqrt(3), 100, YELLOW, forward, 0, 0);
  mainBot.setBackClamp(false);


  // // ~~~~~~~~~~~~~~ Alliance Goal ~~~~~~~~~~~~~~~~
  // mainBot.encoderTurnU(130);
  // mainBot.goForward(24*sqrt(3), 100);
  // mainBot.encoderTurnU(90);
  // mainBot.setFrontClamp(true);
  // mainBot.goForward(24, 50);
  // mainBot.setFrontClamp(false);

  // // ~~~~~~~~~~~ Get out of the 15's way~~~~~~~~~~~
  // mainBot.encoderTurnU(0);
  // mainBot.goForward(-24,30);

  return 1;
}

int concurrentTest() {

  // mainBot.arm.initArmPosition();
  // mainBot.arm.moveArmToPosition(ArmGraph::PLATFORM_HEIGHT);
  wait(250, msec);
  mainBot.driveArmDown(2);
  logController("Go down done");

  return 0;

}

void logHeading(std::string label) {
  static int l = 1;
  log(l, "%s: %f", label.c_str(), mainBot.gyroSensor.heading());
  l = (l > 9 ? 1 : l + 1);
  // printf("%s: %f", label.c_str(), mainBot.gyroSensor.heading());
}

int worldSkillsAuto() {
  while (!mainBot.calibrationDone);
  mainBot.gyroSensor.setHeading(0, degrees);
  mainBot.setBackClamp(true);
  mainBot.goRadiusCurve(28, -7, true, 100, 0, 0, false, 5);
  mainBot.goForward(-10, 100);
  mainBot.setBackClamp(false);
  mainBot.goForward(-60, 100);
  mainBot.goTurnU(310);
  wait(2, sec);
  mainBot.goTurnU(0);
  mainBot.setFrontClamp(true);
  mainBot.goForward(30, 100, 0, 0, false); //add stopAfter if this is intended behavior
  mainBot.setFrontClamp(false);
  mainBot.goForward(20, 100,0,5,5);
  return 1;
}

void logGyro() {
  mainBot.gyroSensor.resetRotation();
  while (true) {
    log("%f", mainBot.gyroSensor.rotation());
    wait(20, msec);
  }
}

int testing() {
  return 0;
}

void userControl(void) { mainBot.setBrakeType(coast); task controlLoop1(mainTeleop); }

void autonomous() { mainBot.setBrakeType(hold); task auto1(matchAuto); }

int main() {
  Competition.bStopAllTasksBetweenModes = true;


  mainBot.setBackClamp(false);
  mainBot.setFrontClamp(false);

  // Callibrate Gyro
  mainBot.callibrateGyro();
  
  mainBot.resetArmRotation();
  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);
  // testArmValues();

  while (true) {
    wait(20, msec);
  }

}


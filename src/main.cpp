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

  // In teleop, arm should default to this position
  // mainBot.setMaxArmTorque(ARM_CURRENT::HIGH);
  // mainBot.arm.setArmDestination(ArmGraph::ABOVE_GOAL);

  mainBot.setBackClamp(false);
  mainBot.setFrontClamp(false);
  // mainBot.arm.initArmPosition();
  // mainBot.initPot = mainBot.chainBarPot.value(deg);
  // mainBot.arm.setPotInit(mainBot.initPot);

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
  // Box Rush
  mainBot.openClaw();
  mainBot.goVision(54, 100, YELLOW, forward, 0,0);
  mainBot.closeClaw();
  mainBot.setArmPercent(forward, 30);
  mainBot.goForward(-48, 100);

  // Everything else
  // mainBot.turnToAngle(100, 90, true, reverse);
  // mainBot.moveArmToPosition(mainBot.ABOVE_MIDDLE, 100);
  // mainBot.turnAndAlignVision(true, color, 0.1, true);
  // mainBot.moveArmToPosition(mainBot.INTAKING, 100);
  // mainBot.openClaw();
  // mainBot.driveStraight(20, 13);
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
  mainBot.driveStraight(10, 100, reverse, 5, 5);
  mainBot.setBackClamp(false);
  mainBot.driveStraight(60, 100, reverse, 5, 5);
  mainBot.goTurnU(310);
  wait(2, sec);
  mainBot.goTurnU(0);
  mainBot.setFrontClamp(true);
  //mainBot.arm.moveArmToPosition(ArmGraph::INTAKE_TO_PLACE_INTER_3);
  mainBot.driveStraight(30, 100, forward, 0, 0, false);
  mainBot.setFrontClamp(false);
  mainBot.driveStraight(20, 100, forward, 0, 5);
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

void autonomous() { mainBot.setBrakeType(hold); task auto1(worldSkillsAuto); }

int main() {
  Competition.bStopAllTasksBetweenModes = true;


  mainBot.setBackClamp(false);
  mainBot.setFrontClamp(false);

  // Reset location of arm
  mainBot.callibrateGyro();

  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);
  // testArmValues();

  while (true) {
    wait(20, msec);
  }

}


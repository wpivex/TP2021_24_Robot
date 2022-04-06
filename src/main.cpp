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
*/



int matchAuto() {
  
  return 1;
}

int concurrentTest() {

  mainBot.arm.initArmPosition();
  mainBot.arm.moveArmToPosition(ArmGraph::PLATFORM_HEIGHT);
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

// Alignment: Three ticks from tile intersection aligns with outer side of wheel guard. robot facing outside field
int skillsAuto(void) {
  while (!mainBot.calibrationDone);
  // log("%f", mainBot.gyroSensor.heading());
  mainBot.gyroSensor.setHeading(0, degrees);
  logHeading("start");

  mainBot.setMaxArmTorque(ARM_CURRENT::HIGH);
  mainBot.arm.initArmPosition();
  mainBot.setMaxArmTorque(ARM_CURRENT::LOW);

  // initialize function reference for future concurrency
  std::function<bool(void)> armFunc = std::bind(&ArmGraph::armMovementAuton, &mainBot.arm);
  bool badGyro = false;

  // Head towards yellow goal
  mainBot.setBackClamp(true);
  // mainBot.gyroCurve(-20, 100, 30, 5, false);
  // logHeading("before curved drive");
  mainBot.driveCurved(20, 100, reverse, 10, 0, 0.3, false); // arc to goal direction
  // logHeading("before vision drive");
  mainBot.goVision(5, 100, YELLOW, reverse, 0, 5, 5, true);
  // wait(1000, msec);
  // logHeading("before straight drive");
  // mainBot.driveStraight(27, 100, reverse, 5, 0, false);
  mainBot.goForward(-32, 100, 0, 0, 5);
  // logHeading("before clamp");
  mainBot.setBackClamp(false); // clamp center goal
  // logHeading("before drive straight");
  mainBot.driveStraight(5, 100, reverse, 5, 5, true);

  // Align for right goal
  // logHeading("before fwd for right goal");
  mainBot.goForward(33, 100); // go back for approach to second goal
  // logHeading("before turn");
  // set to 322 if wrong
  // if(mainBot.gyroSensor.heading(degrees) < 100) {
  //   mainBot.gyroSensor.setHeading(321.5, degrees);
  //   log(12, "DOING SKETCHY ANGLE RESET");
  //   badGyro = true;
  // }
  mainBot.goTurnFastU(205, 60, 35, 360); // turn to yellow goal quickly concurrent with arm movement
  logHeading("after turn");
  mainBot.setMaxArmTorque(ARM_CURRENT::MID);
  mainBot.arm.moveArmToPosition(ArmGraph::INTAKE_TO_PLACE_INTER_3);
  logHeading("after arm");

  // Grab right goal
  mainBot.setFrontClamp(true); // open front clamp
  mainBot.goVision(26, 100, YELLOW, forward, 0, 0, 5, false, 60); // go to goal. Grabs goal while moving forward
  mainBot.setFrontClamp(false); // clamp goal while moving forward

  // Traverse field to red goal
  mainBot.driveStraight(17, 100, forward, 5, 0, false); // robot does not stop after function so momentum carries over to driveCurved
  // mainBot.driveCurved(20, 100, forward, 5, 5, -0.15);
  wait(250, msec);
  mainBot.goTurnFast(false, 35, 60, 35, 30); // Turn to approximate location to red goal at a distance

  mainBot.setMaxArmTorque(ARM_CURRENT::HIGH);
  mainBot.arm.moveArmToPosition(ArmGraph::INTAKE, 100, 5);
  mainBot.openClaw();
  mainBot.goVision(13, 30, RED, forward, 0, 5, 3, true, 120); // Approach red goal
  // mainBot.goTurnFast(true, 3, 40, 40, 0);
  mainBot.driveArmDown(1);
  mainBot.arm.moveArmToPosition(ArmGraph::INTAKE, 100, 5); // bring platform down

  // Pick up red goal  
  mainBot.goVision(8, 30, RED, forward, 0, 5, 2, true, 120); // Approach red goal
  mainBot.closeClaw(); // grab red goal
  wait(100, msec);
  mainBot.driveStraight(2, 20, reverse, 2, 3);

  // Back off and go back home
  mainBot.setMaxDriveTorque(ARM_CURRENT::LOW);
  mainBot.arm.moveArmToPosition(ArmGraph::INTAKE_TO_PLACE_INTER_2, 100, 3);
  mainBot.setMaxDriveTorque(ARM_CURRENT::HIGH);
  wait(500, msec);
  mainBot.driveStraight(3, 40, reverse, 2, 2, true); // move back while concurrently raising goal
  // if(badGyro) {
  mainBot.setMaxArmTorque(ARM_CURRENT::MID);
  mainBot.goTurnFast(false, 150, 100, 40, 30, 0, 5);

  // } else {
  //   mainBot.goTurnFastU(350, 100, 35, 40, 0, 3); // aim back towards platform, parallel to field
  // }

  // TURN TO BLUE
  mainBot.goVision(12, 100, BLUE, forward, 0, 0, 5, false);
  mainBot.goTurnFast(true, 40, 70, 40, 30, 0, 5);
  mainBot.driveStraight(15, 50, forward, 5, 5);
  if(badGyro) {
    mainBot.goTurnFast(false, 30, 70, 40, 30, 0, 5);
  } else {
    mainBot.goTurnFastU(0, 70, 30, 360);
  }
  mainBot.goForward(48, 100);

  // Drive to other side
  mainBot.driveStraightTimed(30, forward, 2); // localize with home wall

  // localize position
  mainBot.driveStraight(2.5, 20, reverse, 2, 3); // back off for clearance to be able to turn
  if(badGyro) {
    mainBot.goTurnFast(true, 90, 70, 40, 30, 0, 5);
  } else {
    mainBot.goTurnFastU(90, 70, 30, 90);
  }
  mainBot.driveStraightTimed(30, reverse, 3); // localize with side wall

  mainBot.driveStraight(4, 40, forward, 5, 3);
  mainBot.goTurnFast(false, 25, 70, 40, 30, 0, 5);
  mainBot.driveStraight(8, 50, forward, 5, 0);
  if(badGyro) {
    mainBot.goTurnFast(true, 10, 70, 40, 30, 0, 5);
  } else {
    mainBot.goTurnFastU(80, 40, 30, 360);
  }
  mainBot.arm.moveArmToPosition(ArmGraph::INTAKE, 100, 5); // bring platform down
  mainBot.fourBarRight.setBrake(coast);
  mainBot.fourBarLeft.setBrake(coast);
  mainBot.setMaxArmTorque(ARM_CURRENT::OFF); // divert all current to motors
  mainBot.goForward(51, 100, 0, 0, 100);
  wait(500, msec);
  mainBot.goForward(-2, 50, 0, 0, 5);

  /*
  // Climb
  while (fabs(levelPitch-mainBot.gyroSensor.pitch()) > 5) {
    logController("%f %f", levelPitch, mainBot.gyroSensor.pitch());
    mainBot.driveStraight(1, 50, forward, 3, 0);
    wait(1000, msec);
  }
  */

  return 1;
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
  mainBot.arm.moveArmToPosition(ArmGraph::INTAKE_TO_PLACE_INTER_3);
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


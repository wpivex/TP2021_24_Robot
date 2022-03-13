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
  mainBot.setMaxArmTorque(ARM_CURRENT::HIGH);
  mainBot.arm.setArmDestination(ArmGraph::ABOVE_GOAL);

  mainBot.setBackClamp(false);
  mainBot.setFrontClamp(false);
  mainBot.arm.initArmPosition();
  mainBot.initPot = mainBot.chainBarPot.value(deg);
  mainBot.arm.setPotInit(mainBot.initPot);

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

void oldFourGoalSkills() {

  mainBot.openClaw();
  mainBot.arm.moveArmToPosition(ArmGraph::ABOVE_GOAL);
  mainBot.driveCurved(20, 100, reverse, 5, 0, 0.33, true);
  mainBot.goVision(18, 30, YELLOW, reverse, 0, 5);
  mainBot.arm.moveArmToPosition(ArmGraph::INTAKE);
  mainBot.goForward(-7, 20);
  mainBot.closeClaw();

  // Place center goal
  mainBot.arm.moveArmToPosition(ArmGraph::ABOVE_GOAL);
  // mainBot.turnToAngle(100, 125, true, reverse); // UNIVERSAL
  mainBot.goForward(20, 60);
  mainBot.arm.moveArmToPosition(ArmGraph::PLATFORM_HEIGHT, 70);
  mainBot.openClaw();

  // Get right goal
  mainBot.goForward(-5, 100);
  // mainBot.turnToAngle(100, 160, true, reverse); // UNIVERSAL
  // mainBot.alignToGoalVision(YELLOW, false, forward, 5, 20);
  mainBot.setFrontClamp(true);
  wait(125, msec);
  // mainBot.goForwardVision(YELLOW, 100, forward, 26, 5, nullptr, nullptr, 0.35);
  mainBot.setFrontClamp(false);
  wait(125,msec);

  log("Part 4: Red Dead Redemption");
  // mainBot.driveStraightGyro(15, 100, reverse, 5, 5);
  // // mainBot.turnToUniversalAngleGyro(0, 30, 180, 10);
  // mainBot.gyroTurnU(0);
  // // mainBot.driveStraightGyro(20, 100, forward, 5, 5,nullptr);
  // // // mainBot.turnToUniversalAngleGyro(270, 20, 30, 10);
  // // mainBot.gyroTurnU(270);
  // mainBot.driveStraight(35, 40, forward, 5, 20);
  // // mainBot.turnToAngleGyro(true, 90, 50, 90, 5);
  
  // log("Part 5: Claw Machine");
  // mainBot.alignToGoalVision(RED, true, forward, 10, 20);  
  // mainBot.driveStraightGyro(3, 20, forward, 5, 3,nullptr);
  // mainBot.arm.moveArmToPosition(ArmGraph::INTAKE);
  // mainBot.openClaw();
  // mainBot.driveStraightGyro(2, 20, forward, 5, 2,nullptr);
  // mainBot.closeClaw();

  log("Part 6: Sluuuuuuuurrrrrrrrppppppppp");
  mainBot.arm.moveArmToPosition(ArmGraph::INTAKE_TO_PLACE_INTER_3);
  mainBot.driveStraight(10, 20, reverse, 5, 10);





  // // Place right goal
  // mainBot.turnToAngle(100, -147, true, forward); // UNIVERSAL
  // mainBot.goForward(24, 20);
  // mainBot.arm.moveArmToPosition(ArmGraph::PLATFORM_HEIGHT, 65);
  // mainBot.openClaw();

  // // Get red goal
  // mainBot.goForward(10, 100);
  // // mainBot.turnToAngle(100, -110, true, forward); // UNIVERSAL
  // mainBot.turnAndAlignVision(true, 1, 0.1, false);
  // mainBot.openClaw();
  // mainBot.goForwardVision(false, 100, 60, 25, 1);
  // mainBot.goVision(16, 20, RED, reverse, 0, 5);
  // mainBot.arm.moveArmToPosition(ArmGraph::INTAKE);
  // mainBot.goForward(5, 20);
  // mainBot.closeClaw();
  // mainBot.arm.moveArmToPosition(ArmGraph::ABOVE_GOAL);

  // // Place red goal
  // // mainBot.turnToAngle(100, 180, true, forward); // UNIVERSAL
  // mainBot.goForward(35, 100);
  // // mainBot.turnToAngle(100, 50, true, forward); // UNIVERSAL
  // mainBot.goForward(35, 25);
  // mainBot.arm.moveArmToPosition(ArmGraph::PLATFORM_HEIGHT, 65);
  // mainBot.openClaw();

  // // Get blue goal
  // mainBot.goForward(5, 100);
  // // mainBot.turnToAngle(100, 90, true, reverse); // UNIVERSAL
  // mainBot.turnAndAlignVision(true, 2, 0.1, false);
  // mainBot.goForwardVision(false, 100, 25, 35, 2);
  // mainBot.arm.moveArmToPosition(ArmGraph::INTAKE, 100);
  // mainBot.goForward(5, 20);
  // mainBot.closeClaw();

  // // Run with blue goal
  // mainBot.arm.moveArmToPosition(ArmGraph::ABOVE_GOAL);
  // // mainBot.turnToAngle(100, 100, true, reverse); // UNIVERSAL
  // mainBot.goForward(50, 100);
}

int matchAuto() {
  // Goal startingPlatformColor = RED;
  // Goal oppositeColor = BLUE;
  
  // //std::function<bool(void)> armFunc = std::bind(&ArmGraph::armMovementAuton, &mainBot.arm);

  // while (mainBot.gyroSensor.isCalibrating()) wait(20, msec);

  // // Drive curved while moving arm to a raised position without blocking vision sensor
  // mainBot.setBackClamp(true);
  // log("Part 1: Starting in the Middle");
  // mainBot.driveCurved(20, 100, reverse, 5, 0, 0.33, true);
  // mainBot.goForwardVision(YELLOW, 50, reverse, 25, 5, nullptr, nullptr, 0.5);
  // mainBot.driveStraightGyro(10, 100, reverse, 5, 5);
  // mainBot.setBackClamp(false);
  // wait(125, msec);
  // log("Part 2: Ctrl+Z");
  // //guarantee the correct angle
  // // mainBot.turnToUniversalAngleGyro(315, 10, 30, 5);
  // mainBot.gyroTurnU(315);
  
  // mainBot.arm.setArmDestination(ArmGraph::INTAKE_TO_PLACE_INTER_2);
  // mainBot.driveStraightGyro(20, 100, forward, 5, 5, nullptr);
  // mainBot.arm.moveArmToPosition(ArmGraph::INTAKE_TO_PLACE_INTER_2);
  // // mainBot.turnToUniversalAngleGyro(225, 20, 30, 10);
  // mainBot.gyroTurnU(225);
  // log("Part 3: The Right Way");

  // mainBot.alignToGoalVision(YELLOW, false, forward, 5, 20);
  // mainBot.setFrontClamp(true);
  // wait(125, msec);
  // mainBot.goForwardVision(YELLOW, 100, forward, 26, 5, nullptr, nullptr, 0.35);
  // mainBot.setFrontClamp(false);
  // wait(125,msec);

  // log("Part 4: Red Dead Redemption");
  // mainBot.driveStraightGyro(15, 100, reverse, 5, 5);
  // // mainBot.turnToUniversalAngleGyro(0, 30, 180, 10);
  // mainBot.gyroTurnU(0);
  // // mainBot.driveStraightGyro(20, 100, forward, 5, 5,nullptr);
  // // // mainBot.turnToUniversalAngleGyro(270, 20, 30, 10);
  // // mainBot.gyroTurnU(270);
  // mainBot.driveStraight(35, 40, forward, 5, 20);
  // // mainBot.turnToAngleGyro(true, 90, 50, 90, 5);
  
  // log("Part 5: Claw Machine");
  // // mainBot.alignToGoalVision(RED, true, forward, 10, 20);  
  // // mainBot.driveStraightGyro(3, 20, forward, 5, 3,nullptr);
  // // mainBot.arm.moveArmToPosition(ArmGraph::INTAKE);
  // // mainBot.openClaw();
  // // mainBot.driveStraightGyro(2, 20, forward, 5, 2,nullptr);
  // // mainBot.closeClaw();

  // // log("Part 6: Sluuuuuuuurrrrrrrrppppppppp");
  // // mainBot.arm.moveArmToPosition(ArmGraph::INTAKE_TO_PLACE_INTER_3);
  // // mainBot.driveStraight(10, 20, reverse, 5, 10);
  
  // log("Fin.");
  return 1;
}


void testArmValues() {
  // mainBot.fourBarLeft.setBrake(coast);
  // mainBot.fourBarRight.setBrake(coast);
  // mainBot.chainBarLeft.setBrake(coast);
  // mainBot.chainBarRight.setBrake(coast);

  // bool wasAPressed = false;
  // bool wasClawPressed = false;
  // bool isClawOpen = false;
  // while (true) {
  //   if (Controller1.ButtonUp.pressing()) {
  //     if(!wasClawPressed) {
  //       mainBot.claw.rotateTo(isClawOpen? 500 : -520, deg, 100, velocityUnits::pct, false);
  //       isClawOpen = !isClawOpen;
  //     }
  //     wasClawPressed = true;
  //   } else {
  //     wasClawPressed = false;
  //   }
  //   if (Controller1.ButtonA.pressing()) {
  //     if(!wasAPressed) {
  //       printf("{%f, %f},\n",  mainBot.fourBarRight.position(degrees), mainBot.chainBarRight.position(degrees));
  //     }
  //     wasAPressed = true;
  //   } else {
  //     wasAPressed = false;
  //   }
  //   wait(20, msec);
  // }
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

void logGyro() {
  mainBot.gyroSensor.resetRotation();
  while (true) {
    log("%f", mainBot.gyroSensor.rotation());
    wait(20, msec);
  }
}

int testing() {
  // mainBot.driveStraight(13, 40, reverse, 2, 3, true); // move back while concurrently raising goal
  // // mainBot.goTurnFastU(350, 50, 28, 40, 0, 5); // aim back towards platform, parallel to field
  // mainBot.goTurn(-160);
  // mainBot.goVision(38, 100, BLUE, forward);
  // mainBot.goTurn(40);
  // mainBot.goForward(26, 100);
  // mainBot.goTurn(-33);
  // mainBot.goForward(20, 100);
  // mainBot.driveStraightTimed(30, forward, 3); // localize with home wall
  mainBot.goVision(13, 30, RED, forward, 0, 5, 3, true, 120); // Approach red goal
  return 0;
}

void userControl(void) { mainBot.setBrakeType(coast); task controlLoop1(mainTeleop); }

void autonomous() { mainBot.setBrakeType(hold); task auto1(skillsAuto); }

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


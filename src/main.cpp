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
  mainBot.arm.setArmDestination(ArmGraph::ABOVE_GOAL);

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
  mainBot.fourBarLeft.setBrake(coast);
  mainBot.fourBarRight.setBrake(coast);
  mainBot.chainBarLeft.setBrake(coast);
  mainBot.chainBarRight.setBrake(coast);

  bool wasAPressed = false;
  bool wasClawPressed = false;
  bool isClawOpen = false;
  while (true) {
    if (Controller1.ButtonUp.pressing()) {
      if(!wasClawPressed) {
        mainBot.claw.rotateTo(isClawOpen? 500 : -520, deg, 100, velocityUnits::pct, false);
        isClawOpen = !isClawOpen;
      }
      wasClawPressed = true;
    } else {
      wasClawPressed = false;
    }
    if (Controller1.ButtonA.pressing()) {
      if(!wasAPressed) {
        printf("{%f, %f},\n",  mainBot.fourBarRight.position(degrees), mainBot.chainBarRight.position(degrees));
      }
      wasAPressed = true;
    } else {
      wasAPressed = false;
    }
    wait(20, msec);
  }
}

void concurrentTest() {

  std::function<bool(void)> armFunc = std::bind(&ArmGraph::armMovementAuton, &mainBot.arm);

  mainBot.arm.setArmDestination(ArmGraph::INTAKE);
  mainBot.driveStraightGyro(40, 20, reverse, 20, 10, armFunc);
  mainBot.arm.moveArmToPosition(ArmGraph::INTAKE);

}

int skillsAuto(void) {

  while (mainBot.gyroSensor.isCalibrating()) wait(20, msec);

  // std::function<bool(void)> armFunc = std::bind(&ArmGraph::armMovementAuton, &mainBot.arm);
  mainBot.setBackClamp(true);
  log("Part 1: Starting in the Middle");
  mainBot.driveCurved(20, 100, reverse, 10, 0, 0.33, true);
  
  mainBot.goForwardVision(YELLOW, 50, reverse, 25, 10, nullptr, nullptr, 0.5);
  mainBot.driveStraightGyro(10, 100, reverse, 5, 5);

  mainBot.setBackClamp(false);
  wait(250, msec);
  log("Part 2: Ctrl+Z");
  //guarantee the correct angle
  // mainBot.turnToUniversalAngleGyro(315, 10, 30, 5);
  mainBot.gyroTurnU(315);
  
  mainBot.driveStraightGyro(22, 100, forward, 5, 5);
  log("Part 3: Spaghetti Arm");
  mainBot.arm.setArmDestination(ArmGraph::INTAKE_TO_PLACE_INTER_2);
  // mainBot.turnToUniversalAngleGyro(225, 20, 30, 10);
  mainBot.gyroTurnU(223);
  mainBot.arm.moveArmToPosition(ArmGraph::INTAKE_TO_PLACE_INTER_2, 100);
  log("Part 4: The Right Way");

  mainBot.alignToGoalVision(YELLOW, false, forward, 10);
  mainBot.setFrontClamp(true);
  wait(250, msec);
  mainBot.goForwardVision(YELLOW, 100, forward, 26, 10, nullptr, nullptr, 0.35);
  mainBot.setFrontClamp(false);
  wait(250,msec);
  

  //Note: Parts 1-4 are the same for skills and normal match autons. 

  log("Part 5: Red Dead Redemption");

  mainBot.driveStraightGyro(18, 40, forward, 5, 5);
  // mainBot.turnToUniversalAngleGyro(180, 20, 30, 10);
  mainBot.gyroTurnU(180);
  mainBot.driveStraight(10, 20, forward, 5, 5);
  // mainBot.turnToUniversalAngleGyro(150, 20, 30, 10);
  mainBot.gyroTurnU(150);
  mainBot.alignToGoalVision(RED, true, forward, 5, 10);
  mainBot.goForwardVision(RED, 40, forward, 7, 10, nullptr, nullptr, 0.2);
  mainBot.alignToGoalVision(RED, true, forward, 5, 10);
  
  log("Part 6: Spaghetti Arm 2: Revenge of Linguini");
  mainBot.openClaw();
  mainBot.arm.moveArmToPosition(ArmGraph::INTAKE, 100);
  mainBot.driveStraightGyro(5, 20, forward, 5, 10);
  mainBot.closeClaw();
  mainBot.driveStraight(5, 20, reverse, 2, 3);
  mainBot.arm.moveArmToPosition(ArmGraph::INTAKE_TO_PLACE_INTER_1, 100);

  log("Part 7: Filthy Acts At A Reasonable Price");
  mainBot.gyroTurnU(25);
  
  // mainBot.turnToUniversalAngleGyro(330, 20, 30, 3);
  
  // mainBot.alignToGoalVision(BLUE, true, forward, 10, 20);
  // // mainBot.turnToAngleGyro(true, 7, 10, 35, 5); //angle slightly off, much better now with setHeading
  // mainBot.gyroTurn(true, 7);
  // mainBot.gyroSensor.setHeading(0, degrees); //this works somehow
  mainBot.driveStraight(50, 100, forward, 7, 24); // I no longer trust gyro straight for long distances
  mainBot.driveStraightTimed(30, forward, 4);
  mainBot.arm.moveArmToPosition(ArmGraph::PLATFORM_HEIGHT, 100);
  mainBot.openClaw();
  mainBot.arm.moveArmToPosition(ArmGraph::ABOVE_GOAL,100);
  mainBot.driveStraight(3, 20, reverse, 2, 3);
  mainBot.gyroTurnU(90);
  mainBot.driveStraightGyro(48, 100, forward, 5, 16);

  // mainBot.turnToUniversalAngleGyro(90, 20, 30, 3);
  //mainBot.gyroTurnU(90);
  // wait(2000, msec);

  log("Part 8: Where The Fuck Am I");
  // mainBot.driveCurved(35, 20, reverse, 10, 0, 0.15);
  // mainBot.turnToUniversalAngleGyro(90, 10, 10, 5);
  // mainBot.driveStraightGyro(10,20,forward,10,10,nullptr);
  // mainBot.turnToUniversalAngleGyro(93, 10, 10, 5);
  // mainBot.arm.setArmDestination(ArmGraph::INTAKE);
  // mainBot.arm.armMovementAuton();
  //mainBot.driveStraight(36,50,forward,10,10,nullptr);
  
  
  // setFrontClamp(true);
  // moveArmToPosition(PLACE_GOAL, 100);
  // openClaw();
  // moveArmToPosition(ABOVE_MIDDLE, 100);
  // setFrontClamp(false);

  // turn and climb
  return 1;
}

int vcatTesting() {
  mainBot.setFrontClamp(false);
  mainBot.setBackClamp(false);
  // mainBot.turnToUniversalAngleGyro(90, 30, 90, 10);
  mainBot.gyroTurnU(90);
  wait(1000,msec);
  // mainBot.turnToUniversalAngleGyro(180, 30, 90, 10);
  mainBot.gyroTurnU(180);
  wait(1000,msec);
  // mainBot.turnToUniversalAngleGyro(270, 30, 90, 10);
  mainBot.gyroTurnU(270);
  wait(1000,msec);
  // mainBot.turnToUniversalAngleGyro(90, 30, 180, 5);
  mainBot.gyroTurnU(90);
  wait(1000,msec);
  return 0;
}


void logGyro() {
  mainBot.gyroSensor.resetRotation();
  while (true) {
    log("%f", mainBot.gyroSensor.rotation());
    wait(20, msec);
  }
}

void userControl(void) { task controlLoop1(mainTeleop); }

void autonomous() { task auto1(matchAuto); }

int main() {
  Competition.bStopAllTasksBetweenModes = true;

  mainBot.setBackClamp(false);
  mainBot.setFrontClamp(false);

  // Reset location of arm
  mainBot.arm.initArmPosition();
  mainBot.callibrateGyro();

  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);
  // testArmValues();

  while (true) {
    wait(20, msec);
  }

}


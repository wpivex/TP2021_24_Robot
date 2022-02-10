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

void userControl(void) { task controlLoop1(mainTeleop); }

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

void skillsVCAT() {
  Goal startingPlatformColor = RED;
  Goal oppositeColor = BLUE;


  // a function reference to mainBot arm's armMovement() function
  std::function<bool(void)> armFunc = std::bind(&ArmGraph::armMovementAuton, &mainBot.arm); 

  // Drive curved while moving arm to a raised position without blocking vision sensor
  mainBot.arm.setArmDestination(ArmGraph::ABOVE_GOAL);
  mainBot.driveCurved(15, 100, forward, 10, 0, -0.65, true, armFunc); // distance, speed, direction, seconds, slowdown, turn, stopafter, function
  mainBot.arm.moveArmToPosition(ArmGraph::ABOVE_GOAL);
  
  // Go to center goal
  mainBot.goForwardVision(YELLOW, 50, forward, 16, 10, nullptr, nullptr, 1.5);
  mainBot.openClaw(false); // open claw non blocking
  mainBot.arm.moveArmToPosition(ArmGraph::INTAKE);
  
  mainBot.driveStraightGyro(14, 20, forward, 10, 0);
  mainBot.closeClaw();

  // // Go to platform and elevate center goal on platform
  mainBot.arm.moveArmToPosition(ArmGraph::INTAKE_TO_PLACE_INTER_2); // wait for arm to lift off the ground for clearance
  mainBot.turnToUniversalAngleGyro(195, 100, 20, 10); // turn to platform, slowdown at 20 degrees to destination
  
  // TODO: limit switch detection
  
  mainBot.driveStraightGyro(20, 60, forward, 10, 10); // Head to platform, slowdown at 10 inches to destination
  mainBot.arm.moveArmToPosition(ArmGraph::PLATFORM_HEIGHT, 40); // Lower goal to paltform
  wait(250, msec);
  mainBot.openClaw(); // place goal

  // Get right goal
  mainBot.driveStraightGyro(5, 100, reverse, 10, 2); // go back a little for clearance
  mainBot.turnToUniversalAngleGyro(30, 100, 10, 10, armFunc); // turn to right goal while moving arm to intaking
  mainBot.alignToGoalVision(YELLOW, false, forward, 5);
  mainBot.arm.moveArmToPosition(ArmGraph::INTAKE); // finish the arm movement to intaking if it's not done
  // TODO: limit switch detection
  mainBot.goForwardVision(YELLOW, 18, forward, 16, 10, nullptr);
  mainBot.closeClaw();

  // Go to platform and place right goal
  mainBot.arm.setArmDestination(ArmGraph::INTAKE_TO_PLACE_INTER_2); // a little above platform level
  mainBot.turnToUniversalAngleGyro(210, 100, 20, 10, armFunc); // turn to platform, slowdown at 20 degrees to destination
  mainBot.driveStraightGyro(24, 20, forward, 10, 10, armFunc); // Head to platform, slowdown at 10 inches to destination
  mainBot.arm.moveArmToPosition(ArmGraph::PLATFORM_HEIGHT); // Lower goal to paltform
  mainBot.openClaw(); // place goal

  // Get blue goal
  mainBot.driveStraightGyro(5, 100, reverse, 10, 2); // go back a little for clearance
  mainBot.arm.setArmDestination(ArmGraph::INTAKE); // to prepare to obtain blue goal
  mainBot.turnToUniversalAngleGyro(90, 100, 30, 10);
  // TODO: limit switch detection
  mainBot.goForwardVision(oppositeColor, 80, forward, 20, 10, nullptr, armFunc); // Travel the majority of the route to blue fast
  mainBot.arm.moveArmToPosition(ArmGraph::INTAKE); // finish moving arm to intake if not done
  mainBot.goForwardVision(oppositeColor, 30, forward, 10, 10, nullptr);
  mainBot.closeClaw();

  // Align with right wall
  mainBot.arm.moveArmToPosition(ArmGraph::INTAKE_TO_PLACE_INTER_3); // move arm up for enough clearance to wall align
  mainBot.driveStraightTimed(60, forward, 2);
  mainBot.driveStraightGyro(10, 100, reverse, 10, 2); // go back a little for clearance

  // Head towards other side of field to drop red goal off and pick up blue goal
  mainBot.turnToUniversalAngleGyro(0, 100, 30, 10);
  mainBot.arm.setArmDestination(ArmGraph::PLATFORM_HEIGHT); // to prepare to drop off red goal
  mainBot.driveStraightGyro(60, 100, forward, 10, 10, armFunc); // cross field
  mainBot.openClaw(); // drop red goal
  mainBot.driveStraightGyro(10, 80, reverse, 5, 5);

  // Turn to red and pick it up
  mainBot.arm.setArmDestination(ArmGraph::INTAKE);
  mainBot.turnToAngleGyro(true, 45, 80, 10, 5, armFunc); // initial turn to red
  mainBot.turnToUniversalAngleGyro(315, 100, 10, 10);
  mainBot.arm.moveArmToPosition(ArmGraph::INTAKE); // finish lowering arm to intake if not done
  mainBot.alignToGoalVision(startingPlatformColor, false, forward, 5); // align to red
  // TODO: limit switch detection
  mainBot.goForwardVision(startingPlatformColor, 30, forward, 20, 10, nullptr); // go to red
  mainBot.closeClaw();

  // Drag red out of platform slowly without raising arm (which would tip platform)
  mainBot.driveStraight(15, 40, reverse, 10, 5);
  mainBot.arm.moveArmToPosition(ArmGraph::PLATFORM_HEIGHT);

  // wall align with top wall
  mainBot.turnToAngleGyro(true, 180, 100, -1, 5);
  mainBot.turnToUniversalAngleGyro(180, 100, 30, 10);
  mainBot.driveStraightTimed(60, reverse, 2);

  // Head back home with red
  mainBot.driveStraightGyro(60, 100, forward, 10, 10);
  
}

/*
void skills() {
  // Get center goal
  mainBot.openClaw();
  mainBot.moveArmToPosition(mainBot.ABOVE_MIDDLE, 100);
  mainBot.driveCurved(forward, 20, -54);
  mainBot.goForwardVision(false, 30, 18, 15, 0);
  mainBot.moveArmToPosition(mainBot.INTAKING, 100);
  mainBot.driveStraight(20, 7);
  mainBot.closeClaw();

  // Place center goal
  mainBot.moveArmToPosition(mainBot.ABOVE_MIDDLE, 100);
  mainBot.turnToAngle(100, 125, true, reverse);
  mainBot.driveStraight(60, 20);
  mainBot.moveArmToPosition(mainBot.PLATFORM_LEVEL, 70);
  mainBot.openClaw();

  // Get right goal
  mainBot.driveStraight(100, -5);
  mainBot.turnToAngle(100, 160, true, reverse);
  mainBot.turnAndAlignVision(false, 0, 0.05, false);
  mainBot.openClaw();
  mainBot.moveArmToPosition(mainBot.INTAKING, 100);
  mainBot.goForwardVision(false, 20, 16, 15, 0);
  mainBot.closeClaw();
  mainBot.moveArmToPosition(mainBot.ABOVE_MIDDLE, 100);

  // Place right goal
  mainBot.turnToAngle(100, -147, true, forward);
  mainBot.driveStraight(20, 24);
  mainBot.moveArmToPosition(mainBot.PLATFORM_LEVEL, 65);
  mainBot.openClaw();

  // Get red goal
  mainBot.driveStraight(100, -10);
  mainBot.turnToAngle(100, -110, true, forward);
  mainBot.turnAndAlignVision(true, 1, 0.1, false);
  mainBot.openClaw();
  mainBot.goForwardVision(false, 100, 60, 25, 1);
  mainBot.moveArmToPosition(mainBot.INTAKING, 100);
  mainBot.driveStraight(20, 5);
  mainBot.closeClaw();
  mainBot.moveArmToPosition(mainBot.ABOVE_MIDDLE, 100);

  // Place red goal
  mainBot.turnToAngle(100, 180, true, forward);
  mainBot.driveStraight(100, 35);
  mainBot.turnToAngle(100, 50, true, forward);
  mainBot.driveStraight(25, 35);
  mainBot.moveArmToPosition(mainBot.PLATFORM_LEVEL, 65);
  mainBot.openClaw();

  // Get blue goal
  mainBot.driveStraight(100, -5);
  mainBot.turnToAngle(100, 90, true, reverse);
  mainBot.turnAndAlignVision(true, 2, 0.1, false);
  mainBot.goForwardVision(false, 100, 25, 35, 2);
  mainBot.moveArmToPosition(mainBot.INTAKING, 100);
  mainBot.driveStraight(20, 5);
  mainBot.closeClaw();

  // Run with blue goal
  mainBot.moveArmToPosition(mainBot.ABOVE_MIDDLE, 100);
  mainBot.turnToAngle(100, 100, true, reverse);
  mainBot.driveStraight(100, 50);
}

int tetherAuto(void) { return 0; }
*/

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

void oldAuton(void) {
  std::function<bool(void)> armFunc = std::bind(&ArmGraph::armMovementAuton, &mainBot.arm);
  mainBot.setBackClamp(true);
  log("Part 1");
  mainBot.driveCurved(20, 100, reverse, 10, 0, 0.33, true);
  wait(250, msec);
  
  mainBot.goForwardVision(YELLOW, 50, reverse, 25, 10, nullptr, nullptr, 0.5);
  mainBot.driveStraight(10, 100, reverse, 5, 5, true);

  mainBot.setBackClamp(false);
  wait(250,msec);
  log("Part 2");
  //guarantee the correct angle
  mainBot.turnToUniversalAngleGyro(325, 100, 30, 5);
  
  mainBot.driveStraight(24, 100, forward, 5, 5, true);
  log("Part 3");
  mainBot.arm.setArmDestination(ArmGraph::INTAKE_TO_PLACE_INTER_3);
  mainBot.turnToUniversalAngleGyro(225, 50, 30, 10, nullptr);
  mainBot.arm.moveArmToPosition(ArmGraph::INTAKE_TO_PLACE_INTER_3, 40);
  wait(1000,msec);
  log("Part 4");
  mainBot.alignToGoalVision(YELLOW, false, forward, 10);
  
  mainBot.setFrontClamp(true);
  mainBot.goForwardVision(YELLOW, 100, forward, 26, 10, nullptr, nullptr, 0.35);
  mainBot.setFrontClamp(false);
  wait(250, msec);
  

  //Note: Parts 1-4 are the same for skills and normal match autons. 

  log("Part 5");

  mainBot.driveCurved(15, 100, forward, 5, 25, true);
  
  mainBot.goForwardVision(RED, 100, forward, 35, 10, nullptr, nullptr, 0.35);
  mainBot.alignToGoalVision(RED, true, forward,5);
  log("Part 6");
  mainBot.arm.moveArmToPosition(ArmGraph::INTAKE, 100);
  mainBot.openClaw();
  mainBot.driveStraightGyro(15, 40, forward, 5, 10);
  mainBot.closeClaw();
  mainBot.arm.moveArmToPosition(ArmGraph::PLACE_GOAL_WITH_YELLOW, 100);
  log("Part 7");
  mainBot.turnToUniversalAngleGyro(180, 55, 20, 5);
  mainBot.driveStraightGyro(96, 100, reverse, 10, 24);
  // setFrontClamp(true);
  // moveArmToPosition(PLACE_GOAL, 100);
  // openClaw();
  // moveArmToPosition(ABOVE_MIDDLE, 100);
  // setFrontClamp(false);

  // turn and climb
}

void vcatTesting() {
  //mainBot.driveStraight(50, 80, forward, 20, 10);
  //mainBot.driveStraightGyro(80, 80, forward, 20, 10);
  // mainBot.callibrateGyro();
  // mainBot.turnToAngleGyro(true, 90, 100, 30, 20);
  // while(true) {
  //   log(mainBot.gyroSensor.rotation());
  // }
  // mainBot.driveStraightTimed(50, forward, 2, true);
  // mainBot.driveStraight(300, 100, forward, 1, 0);
  // mainBot.driveCurved(30, 80, forward, 1000, 10, 0.5);
  // mainBot.goForwardVision(RED, 20, forward, 40, 100000, nullptr);
  // mainBot.alignToGoalVision(YELLOW, false, forward, 10000);
  // mainBot.arm.setArmPosition(ArmGraph::PLATFORM_HEIGHT);
  mainBot.arm.moveArmToPosition(ArmGraph::PLACE_GOAL_WITH_YELLOW);
  //mainBot.arm.moveArmToPosition(ArmGraph::INTAKE_TO_PLACE_INTER_3);
  mainBot.arm.moveArmToPosition(ArmGraph::PLATFORM_HEIGHT);
}

void autonomous() { thread auto1(oldAuton); }

void logGyro() {
  mainBot.gyroSensor.resetRotation();
  while (true) {
    log("%f", mainBot.gyroSensor.rotation());
    wait(20, msec);
  }
}

int main() {

  // mainBot.callibrateGyro(); 
  // testArmValues(); 

  // logGyro();

  // mainBot.setBackClamp(false);
  // mainBot.setFrontClamp(false);

  // // Reset location of arm
  mainBot.arm.initArmPosition();
  mainBot.callibrateGyro();

  // vcatTesting();

  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);
  // testArmValues();

  // test concurrent
  //concurrentTest();


  while (true) {
    wait(20, msec);
  }
}

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

  mainBot.gyroSensor.setHeading(180, degrees); // robot starts facing backwards

  mainBot.setMaxArmTorque(ARM_CURRENT::HIGH);
  mainBot.setMaxDriveTorque(ARM_CURRENT::HIGH);

  // initialize function reference for future concurrency
  std::function<bool(void)> armFunc = std::bind(&ArmGraph::armMovementAuton, &mainBot.arm);

  // Go for middle goal
  mainBot.setBackClamp(true);
  mainBot.arm.setArmDestination(ArmGraph::INTAKE_TO_PLACE_INTER_3);
  mainBot.goForward(-50, 100, 0, -1, false, armFunc); // head towards yellow goal while moving arm
  mainBot.arm.finishArmMovement(); // finish concurrent arm movement
  mainBot.setBackClamp(false); // clamp center goal
  mainBot.goForward(-5, 100, 5); // continue going forward while clamping

  // Go for right goal
  mainBot.goForward(33, 100, 5); // go back for approach to second goal
  mainBot.possiblyResetGyroWithGPS();
  mainBot.goTurnU_TRAP(25);
  mainBot.setFrontClamp(true);
  mainBot.goVision(26, 90, YELLOW, 0, -1, false); // go to yellow goal
  mainBot.setFrontClamp(false); // grab yellow goal
  mainBot.goForwardU(13, 80, 25, 0, -1, false); // Universal used to correct slight vision errors

  // Go for red goal
  mainBot.goCurve(20, 100, -0.1, 5); // continue momentum and curve towards red goal
  mainBot.goTurnU_TRAP(320); // aim approximately at red goal
  mainBot.goVision(13, 30, YELLOW, 1); // go slowly towards red goal while arm is still raised as to not block vision sensor
  mainBot.openClaw();
  mainBot.arm.moveArmToPosition(ArmGraph::INTAKE, 100, 3); // no concurrency here because everything must be done sequentially and precisely
  mainBot.driveArmDown(1); // make sure arm is low and ready to grab red goal.
  mainBot.goForward(10, 40, 1);
  mainBot.closeClaw(); // grab red goal

  // Obtain red goal
  mainBot.setMaxDriveTorque(ARM_CURRENT::LOW); // maximum power to the arm
  mainBot.arm.moveArmToPosition(ArmGraph::INTAKE_TO_PLACE_INTER_2);
  mainBot.setMaxDriveTorque(ARM_CURRENT::HIGH); // restore power to drive
  mainBot.goForwardU(13, 40, 320, 2); // back away while trying to minimize future error by approaching the original 320 degree heading
  
  // Go to home platorm area
  mainBot.goTurnU_TRAP(150);
  mainBot.possiblyResetGyroWithGPS();
  mainBot.goTurnU_TRAP(180); // point towards home
  mainBot.goForwardU(82, 90, 180, 2, 30, false); // Go home while maintaing heading for perfect orthogonal movement

  // Localize with two walls
  mainBot.goForwardTimed(2, 30); // align with platform wall
  mainBot.goForwardU(-3, 50, 180, 0.5);
  mainBot.goTurnU_TRAP(270);
  mainBot.goForwardTimed(3, -30); // align with side wall

  // S maneuever to get closer to left side
  mainBot.goCurve(6, 50, -0.3, 0.5);
  mainBot.goCurve(6, 50, -0.3, 0.5);

  // Go on platform. Start by aiming slightly left to strafe to right lateral plane and then self correct to orthogonal
  mainBot.goCurve(10, 50, -0.1, 0.5);
  mainBot.goForward(5, 50, 2);
  mainBot.arm.moveArmToPosition(ArmGraph::INTAKE, 100, 3); // bring platform down
  mainBot.fourBarRight.setBrake(coast); // make sure vex releases those motors!
  mainBot.fourBarLeft.setBrake(coast);
  mainBot.setMaxArmTorque(ARM_CURRENT::OFF); // divert all current to drive motors

  // Balance on platform
  mainBot.goForwardU(51, 90, 270, 0); // self correct for any errors in yaw
  wait(500, msec);
  mainBot.goForward(-2, 100, 0); // sudden jerk back to help balance
  

  return 1;
}

void logGyro() {
  mainBot.gyroSensor.resetRotation();
  while (true) {
    log("%f", mainBot.gyroSensor.rotation());
    wait(20, msec);
  }
}


void userControl(void) { mainBot.setBrakeType(coast); task controlLoop1(mainTeleop); }

void autonomous() { mainBot.setBrakeType(hold); task auto1(skillsAuto); }

int main() {
  Competition.bStopAllTasksBetweenModes = true;
  mainBot.setBackClamp(false);
  mainBot.setFrontClamp(false);

  wait(500, msec);
  mainBot.gyroSensor.calibrate();
  mainBot.waitGyroCallibrate();
  mainBot.arm.initArmPosition();


  

  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);
  // testArmValues();

  while (true) {
    wait(20, msec);
  }

}


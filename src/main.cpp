// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Vision6              vision        6               
// DigitalInE           digital_in    E               
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "../include/robot.cpp"
#include "VisualGraph.cpp"

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
  mainBot.openClaw();
  // Drive forwards at full speed (while adjusting towards goal if needed)
  mainBot.setArmDegrees(5, 50, false);
  mainBot.goForward(36, 100, 3, 20, 5); 
  mainBot.closeClaw();
  wait(200, msec);
  // Raise arm a bit (so that other team cannot grab it)
  mainBot.setArmDegrees(215);
  // RETREAT
  mainBot.goForward(-8, 100, 0, 5); 

  // // ~~~~~~~~~~~ Middle Goal Check ~~~~~~~~~~~~~~
  //mainBot.cursedTurn(150,70);
  mainBot.goTurn(120);
  mainBot.setBackClamp(true);
  mainBot.goVision(50, 65, YELLOW, reverse, 0, 0);
  mainBot.setBackClamp(false);
  wait(200, msec);

  mainBot.goForward(40, 100, 5, 0, 5, {}, false);
  mainBot.goCurve(20, 100, 0.2, 0, 10); // get back to base


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

  return 0;
}

int testCurrent() {

  mainBot.setMaxArmTorque(ARM_CURRENT::MID);
  mainBot.setLeftVelocity(reverse, 100);
  mainBot.setRightVelocity(reverse, 100);
  int startTime = timer::system();
  VisualGraph g(0, 3, 10, 200);
  int a = 0;
  while (!isTimeout(startTime, 2.5)) {
    
    float curr = (mainBot.leftMotorA.current() + mainBot.rightMotorA.current()) / 2;
    g.push(curr);

    a = (a + 1) % 10;
    if (a == 0) g.display();

    wait(20, msec);

  }
  mainBot.stopLeft();
  mainBot.stopRight();

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


// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Vision6              vision        6               
// DigitalInE           digital_in    E               
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "../include/robot.cpp"

competition Competition;
controller Controller1(controllerType::primary);
brain Brain;

Robot mainBot = nullptr;

int mainTeleop() {
  while (true) {
    mainBot.teleop();
    wait(100, msec);
  }
  return 0;
}

void userControl(void) { task controlLoop1(mainTeleop); }

void mainAuto(void) {


  
  Brain.Screen.render(true,false);
  Brain.Screen.clearLine(0,color::black);
  Brain.Screen.clearLine(1,color::black);
  Brain.Screen.clearLine(2,color::black);
  Brain.Screen.clearLine(3,color::black);
  Brain.Screen.clearLine(4,color::black);
  Brain.Screen.clearLine(6,color::black);
  Brain.Screen.clearLine(8,color::black);
  Brain.Screen.setCursor(1,1);
  Brain.Screen.print("Test");
  Brain.Screen.render();
  
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
  mainBot = Robot(&Controller1); 

  /*

  // Reset location of arm
  mainBot.initArm();

  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);

  // testArmValues();

  */

  digital_out a = digital_out(Brain.ThreeWirePort.A);
  digital_in b = digital_in(Brain.ThreeWirePort.B);
  
  long duration;
  int dist;


  int i = 0;

  while (true) {

    
    a.set(false);
    wait(0.002,msec);
    a.set(true);
    wait(0.010,msec);
    a.set(false);
    
    duration = b.value();
    // Calculating the distance
    dist = duration * 0.034 / 2.0 / 2.54; // Speed of sound wave divided by 2 (go and back)

    i++;
    if (i == 1000) {
      i = 0;
      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1, 1);
      Brain.Screen.print("a: %d b: %d", a.value(), b.value());
    }
    
    


  }
}

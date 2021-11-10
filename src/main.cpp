// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Vision6              vision        6               
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

float CENTER_X = 157.0;

bool inBounds(int x, int y,int leftBound, int rightBound, int bottomBound, int topBound) {
  return (x >= leftBound && x <= rightBound && y <= bottomBound && y >= topBound);
}

void goForwardVision(int forwardDistance) {

  mainBot.camera.setBrightness(13);
  vision::signature SIG_1 (1, 1525, 1915, 1720, -3519, -2841, -3180, 5.400, 0);

  mainBot.leftMotorA.resetPosition();
  mainBot.rightMotorA.resetPosition();

  float totalDist = mainBot.distanceToDegrees(forwardDistance);
  float dist = 0;

  while(dist < totalDist) {
    mainBot.camera.takeSnapshot(SIG_1);
    vision::object largestObject;
    int largestArea = -1;

    safearray<vex::vision::object, 16> *objects;
    objects = &mainBot.camera.objects;

    // Find the largest object from vision within the specified bounds
    for(int i=0; i<mainBot.camera.objectCount; i++) {

      // Get the pointer to the current object
      vex::vision::object o = (* objects)[i];
      int area = o.width * o.height;

      // Left bound, right bound, bottom bound, top bound. Check if object within bounds and is largest than current largest object
      if (inBounds(o.centerX,o.centerY,0,236,211, 0) && area > largestArea) {
        largestObject = o;
        largestArea = area;
      }
    }

    float pMod = 25.0;
    float baseSpeed = -100.0+pMod;

    float mod;

    // // if object exists
    if(true || largestArea != -1) {
      // Brain.Screen.setCursor(8,1);
      // Brain.Screen.print(((CENTER_X-mainBot.camera.largestObject.centerX)/CENTER_X*pMod));
      mod = (CENTER_X-mainBot.camera.largestObject.centerX)/CENTER_X*pMod;
    } else {
      mod = 0; // emergency code. if nothing detected just move forwards straight.
    }
    mainBot.setLeftVelocity(forward, baseSpeed+mod);
    mainBot.setRightVelocity(forward, baseSpeed-mod);
    
    Brain.Screen.render(true,false);
    Brain.Screen.clearLine(0,color::black);
    Brain.Screen.clearLine(1,color::black);
    Brain.Screen.clearLine(2,color::black);
    Brain.Screen.clearLine(3,color::black);
    Brain.Screen.clearLine(4,color::black);
    Brain.Screen.clearLine(6,color::black);
    Brain.Screen.clearLine(8,color::black);
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("Largest object: %f, %f", ((double)mainBot.camera.largestObject.centerX)/315, ((double)mainBot.camera.largestObject.centerY)/211);
    Brain.Screen.setCursor(2,1);
    Brain.Screen.print("Largest object in bounds: %f, %f", ((double)largestObject.centerX)/315, ((double)largestObject.centerY)/211);
    // Brain.Screen.print("Largest area: %f", (double)largestArea);
    Brain.Screen.setCursor(3,1);
    Brain.Screen.print("Width and Height: %f", ((double)mainBot.camera.largestObject.width)/315, ((double)mainBot.camera.largestObject.height)/211);
    Brain.Screen.setCursor(4,1);
    Brain.Screen.print("Count: %d", mainBot.camera.objectCount);
    Brain.Screen.render();

    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print(mainBot.camera.largestObject.centerX);
    //Controller1.Screen.print((*objects).getLength());

    wait(100, msec);

    dist = fabs((mainBot.leftMotorA.position(degrees) + mainBot.leftMotorB.position(degrees)) / 2.0);
  }

  mainBot.stopLeft();
  mainBot.stopRight();
}

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
  
  mainBot.driveCurved(reverse, 34, 45);
  //wait(1000,msec);
  goForwardVision(25);

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

  // Reset location of arm
  mainBot.initArm();
  mainBot.fourBarLeft.setBrake(coast);
  mainBot.fourBarRight.setBrake(coast);
  mainBot.chainBarLeft.setBrake(coast);
  mainBot.chainBarRight.setBrake(coast);

  //Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);

  // testArmValues();

  while (true) {
    // Brain.Screen.clearScreen();
    // Brain.Screen.setCursor(1, 1);
    // Brain.Screen.print("%d %d", (int)mainBot.chainBarLeft.position(degrees), (int)mainBot.fourBarLeft.position(degrees));
    wait(100, msec);
  }
}


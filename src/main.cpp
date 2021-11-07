// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Vision6              vision        6               
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "../include/robot.cpp"

competition Competition;
// controller Controller1(controllerType::primary);
// brain Brain;

Robot *mainBotP;

int mainTeleop() {
  while (true) {
    mainBotP->teleop();
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

  mainBotP->camera.setBrightness(13);
  vision::signature SIG_1 (1, 1525, 1915, 1720, -3519, -2841, -3180, 5.400, 0);

  mainBotP->leftMotorA.resetPosition();
  mainBotP->rightMotorA.resetPosition();

  float totalDist = mainBotP->distanceToDegrees(forwardDistance);
  float dist = 0;

  while(dist < totalDist) {
    mainBotP->camera.takeSnapshot(SIG_1);
    vision::object largestObject;
    int largestArea = -1;

    safearray<vex::vision::object, 16> *objects;
    objects = &mainBotP->camera.objects;

    // Find the largest object from vision within the specified bounds
    for(int i=0; i<mainBotP->camera.objectCount; i++) {

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
      // Brain.Screen.print(((CENTER_X-mainBotP->camera.largestObject.centerX)/CENTER_X*pMod));
      mod = (CENTER_X-mainBotP->camera.largestObject.centerX)/CENTER_X*pMod;
    } else {
      mod = 0; // emergency code. if nothing detected just move forwards straight.
    }
    mainBotP->setLeftVelocity(forward, baseSpeed+mod);
    mainBotP->setRightVelocity(forward, baseSpeed-mod);
    
    Brain.Screen.render(true,false);
    Brain.Screen.clearLine(0,color::black);
    Brain.Screen.clearLine(1,color::black);
    Brain.Screen.clearLine(2,color::black);
    Brain.Screen.clearLine(3,color::black);
    Brain.Screen.clearLine(4,color::black);
    Brain.Screen.clearLine(6,color::black);
    Brain.Screen.clearLine(8,color::black);
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("Largest object: %f, %f", ((double)mainBotP->camera.largestObject.centerX)/315, ((double)mainBotP->camera.largestObject.centerY)/211);
    Brain.Screen.setCursor(2,1);
    Brain.Screen.print("Largest object in bounds: %f, %f", ((double)largestObject.centerX)/315, ((double)largestObject.centerY)/211);
    // Brain.Screen.print("Largest area: %f", (double)largestArea);
    Brain.Screen.setCursor(3,1);
    Brain.Screen.print("Width and Height: %f", ((double)mainBotP->camera.largestObject.width)/315, ((double)mainBotP->camera.largestObject.height)/211);
    Brain.Screen.setCursor(4,1);
    Brain.Screen.print("Count: %d", mainBotP->camera.objectCount);
    Brain.Screen.render();

    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print(mainBotP->camera.largestObject.centerX);
    //Controller1.Screen.print((*objects).getLength());

    wait(100, msec);

    dist = fabs((mainBotP->leftMotorA.position(degrees) + mainBotP->leftMotorB.position(degrees)) / 2.0);
  }

  mainBotP->stopLeft();
  mainBotP->stopRight();



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
  
  mainBotP->driveCurved(reverse, 34, 45);
  //wait(1000,msec);
  goForwardVision(25);

}

int tetherAuto(void) { return 0; }

void autonomous() { thread auto1(mainAuto); }

double **getFileAngles(std::string filename) {
  int byteLen = Brain.SDcard.size(filename.c_str()) +
                10; // just in case this is off for some reason
  unsigned char *c = new unsigned char[byteLen];
  Brain.SDcard.loadfile(filename.c_str(), c, byteLen);

  int lineCount = 0;
  for (int i = 0; i < byteLen; i++) {
    if (i != byteLen - 1 && c[i] == 13 && c[i + 1] == 10) {
      lineCount++;
    }
  }
  double **angles = new double *[lineCount + 1];
  for (int i = 0; i < lineCount; ++i) {
    angles[i] = new double[2];
  }

  std::string s = "";
  int currInd = 0;
  for (int i = 0; i < byteLen; i++) {
    s += c[i];
    if (i != byteLen - 1 && c[i] == 13 && c[i + 1] == 10) {
      // note that this approach needs modification if we have more than two
      // values per line
      int commaIndex = s.find(",");
      angles[currInd][0] = atof(s.substr(0, commaIndex).c_str());
      angles[currInd][1] = atof(s.substr(commaIndex + 1, s.length() - commaIndex + 1).c_str());
      currInd++;
      s = "";
    }
  }
  return angles;
}

void testArmValues() {

  while (true) {

    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(0, 0);
    Controller1.Screen.print("%i %i", (int) mainBotP->fourBarLeft.position(degrees), (int) mainBotP->chainBarLeft.position(degrees));

  }
  

}

int main() {
  Robot mainBot = Robot(&Controller1, getFileAngles("motion_profile.csv"));
  mainBotP = &mainBot;

  // Reset location of arm
  mainBotP->initArm();

  //Competition.autonomous(autonomous);
  //Competition.drivercontrol(userControl);

  testArmValues();

  while (true) {
    wait(100, msec);
  }
}


#include "../include/robot.cpp"

competition Competition;
// controller Controller1(controllerType::primary);
// brain Brain;

Robot *mainBotP;

int mainTeleop() {
  while (true) {
    mainBotP->teleop();
  }
  return 0;
}

void userControl(void) { task controlLoop1(mainTeleop); }
float baseSpeed = 50;
float pMod = 20;

void mainAuto(void) {
  vision::signature SIG_1 (1, 1999, 2599, 2299, -3267, -2737, -3002, 2.500, 0);
  while(true) {
    mainBotP->camera.takeSnapshot(SIG_1);
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
    Brain.Screen.print("Width: %f", ((double)mainBotP->camera.largestObject.width)/315);
    Brain.Screen.setCursor(3,1);
    Brain.Screen.print("Height: %f", ((double)mainBotP->camera.largestObject.height)/211);
    Brain.Screen.setCursor(4,1);
    Brain.Screen.print("Count: %d", mainBotP->camera.objectCount);
    Brain.Screen.setCursor(6,1);
    Brain.Screen.print(mainBotP->camera.largestObject.centerX < 175? "LEFT" : "RIGHT");
    // Brain.Screen.setCursor(3,1);
    // Brain.Screen.print("Snapshot: %d", mainBotP->camera.takeSnapshot(SIG_1));
    Brain.Screen.render(); //push data to the LCD all at once to prevent image flickering

    if(mainBotP->camera.largestObject.exists) {
      // Brain.Screen.setCursor(8,1);
      // Brain.Screen.print(((175.0-mainBotP->camera.largestObject.centerX)/175.0*pMod));
      mainBotP->setLeftVelocity(forward, baseSpeed-((175.0-mainBotP->camera.largestObject.centerX)/175.0*pMod));
      mainBotP->setRightVelocity(forward, baseSpeed+((175.0-mainBotP->camera.largestObject.centerX)/175.0*pMod));
    }
  }
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

// ARM CODE VERSION 2: BUTTONS MAPPED TO ALL STATES
int main() {
  Robot mainBot = Robot(&Controller1);
  mainBotP = &mainBot;

  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);

  while (true) {
    wait(100, msec);
  }
}
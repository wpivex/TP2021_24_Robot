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

float CENTER_X = 157.0;

bool inBounds(int x, int y,int leftBound, int rightBound, int bottomBound, int topBound) {
  return (x >= leftBound && x <= rightBound && y <= bottomBound && y >= topBound);
}

void mainAuto(void) {
  vision::signature SIG_1 (1, 949, 1365, 1157, -4229, -3529, -3879, 7.300, 0);
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
    Brain.Screen.print(mainBotP->camera.largestObject.centerX < CENTER_X? "LEFT" : "RIGHT");
    // Brain.Screen.setCursor(3,1);
    // Brain.Screen.print("Snapshot: %d", mainBotP->camera.takeSnapshot(SIG_1));
    Brain.Screen.render(); //push data to the LCD all at once to prevent image flickering

    vision::object *largestObject;
    int largestArea = -1;

    safearray<vex::vision::object, 16> *objects;
    objects = &mainBotP->camera.objects;

    for(int i=0; i<mainBotP->camera.objectCount; i++) {
      // Get the pointer to the current object
      vex::vision::object *o = &((*objects)[i]);
      int area = o->width * o->height;

      // Left bound, right bound, bottom bound, top bound. Check if object within bounds and is largest than current largest object
      if (inBounds(o->centerX,o->centerY,100,250,20,200) && area > largestArea) {
        largestObject = o;
        largestArea = area;
      }
    }

    float pMod = 25.0;
    float baseSpeed = -100.0+pMod;

    // if object exists
    if(largestObject != nullptr) {
      // Brain.Screen.setCursor(8,1);
      // Brain.Screen.print(((CENTER_X-mainBotP->camera.largestObject.centerX)/CENTER_X*pMod));
      float mod = (CENTER_X-largestObject->centerX)/CENTER_X*pMod;
      mainBotP->setLeftVelocity(forward, baseSpeed-mod);
      mainBotP->setRightVelocity(forward, baseSpeed+mod);
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
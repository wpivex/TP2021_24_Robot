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

void mainAuto(void) {
  for (int i = 0; i < 4; i++) {
    mainBotP->driveStraight(30, 24 + 14.5);
    mainBotP->turnToAngle(30, 90);
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

  // Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);


  /* CSV FORMAT THAT IS BEING USED IN THIS CODE:
  Elements 0-4 is the location of Intake, Intermediate point, Ring front, Ring middle.
  Ring back Element 5 is the location of Place Goal
  */
  double **angles = getFileAngles("motion_profile.csv");

  // Prevent main from exiting with an infinite loop.
  mainBot.chainBarFake.resetPosition();
  mainBot.fourBarFake.resetPosition();

  bool isPressed = false; // Button presses register only at the first frame pressed. Also disallows concurrent presses from different buttons.
  bool arrived = true; // Whether arm has arrived onto a node

  float MARGIN = 50; // margin of error for if robot arm is in vicinity of target node
  float baseSpeed = 30; // Base speed of arm

  int finalIndex = 2; // The immediate default destination from the starting point is to Ring Front (index 2)
  int targetIndex = finalIndex;

  // Store starting location of arm motors for purposes of velocity calculation
  float fourStart = mainBot.fourBarFake.position(degrees);
  float chainStart = mainBot.chainBarFake.position(degrees);


  while (true) {
    wait(100, msec);
  }
}
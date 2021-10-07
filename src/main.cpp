#include "../include/robot.cpp"

competition Competition;
controller Controller1(controllerType::primary);
brain Brain;

Robot* mainBotP;

int mainTeleop() {
  while(true) {
    mainBotP->teleop();
  }
  return 0;
}

void userControl(void) {
  task controlLoop1(mainTeleop);
}

void mainAuto(void) {
  for(int i=0; i<4; i++) {
    mainBotP->driveStraight(30, 24 + 14.5);
    mainBotP->turnToAngle(30, 90);
  }
}

int tetherAuto(void) {
  return 0;
}

void autonomous() {
  thread auto1(mainAuto);
}

double** getFileAngles(std::string filename) {
  int byteLen = Brain.SDcard.size(filename.c_str()) + 10; // just in case this is off for some reason
  unsigned char* c = new unsigned char[byteLen];
  Brain.SDcard.loadfile(filename.c_str(), c, byteLen);

  int lineCount = 0;
  for(int i=0; i<byteLen; i++) { if(i != byteLen - 1 && c[i] == 13 && c[i+1] == 10) { lineCount++; } }
  double** angles = new double*[lineCount+1];
  for(int i = 0; i < lineCount; ++i) { angles[i] = new double[2]; }

  std::string s = "";
  int currInd = 0;
  for(int i=0; i<byteLen; i++) {
    s += c[i];
    if(i != byteLen - 1 && c[i] == 13 && c[i+1] == 10) {
      //note that this approach needs modification if we have more than two values per line
      int commaIndex = s.find(",");
      angles[currInd][0] = atof(s.substr(0, commaIndex).c_str());
      angles[currInd][1] = atof(s.substr(commaIndex + 1, s.length() - commaIndex + 1).c_str());
      currInd++;
      s = "";
    }
  }
  return angles;
}

int main() {
  Robot mainBot = Robot(&Controller1);
  mainBotP = &mainBot;
  // Competition.autonomous(autonomous);
  // Competition.drivercontrol(userControl);

  double** angles = getFileAngles("motion_profile.csv");

  // Prevent main from exiting with an infinite loop.
  mainBot.chainBarFake.setPosition(0, degrees);
  mainBot.fourBarFake.setPosition(0, degrees);
  int targetIndex = 0;
  bool isPressed = false;
  while (true) {
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(0, 0);
    Controller1.Screen.print("%f %f", angles[targetIndex][0], angles[targetIndex][1]);
    // angle for 4 is -1000 736
    // angle for intermediate -1323 1362
    // angle for 6 is -393 969
    // angle for 3 is -1266 497
    // angle for 2 is -942 334
    // which is 138	334
    // angle for 5 is -1505 -307
    // Controller1.Screen.setCursor(1, 0);
    // Controller1.Screen.print(mainBot.chainBarFake.position(degrees));

    mainBot.fourBarFake.rotateTo(-angles[targetIndex][0]*14, degrees, false);
    mainBot.chainBarFake.rotateTo(-angles[targetIndex][1]*10, degrees, false);
    if(Controller1.ButtonRight.pressing()) {
      if(!isPressed && targetIndex < 5) { targetIndex++; }
      isPressed = true;
    } else if(Controller1.ButtonLeft.pressing()) {
      if(!isPressed && targetIndex != 0) { targetIndex--; }
      isPressed = true;
    } else {
      isPressed = false;
    }

    wait(100, msec);
  }
}

#include "../include/robot.cpp"

competition Competition;
controller Controller1(controllerType::primary);
brain Brain;

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
      angles[currInd][1] =
          atof(s.substr(commaIndex + 1, s.length() - commaIndex + 1).c_str());
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

  double **angles = getFileAngles("motion_profile.csv");

  // Prevent main from exiting with an infinite loop.
  mainBot.chainBarFake.resetPosition();
  mainBot.fourBarFake.resetPosition();
  int targetIndex = 2; // The immediate default destination from the starting
                       // point is to Ring Front (index 2)
  bool isPressed = false;
  bool arrived = true;
  bool isPlaceGoal = false;
  bool goingRight =
      false; // Direction arm is travelling right now. For now, only purpose is
             // to decide where to go after intermediate point

  /* CSV FORMAT THAT IS BEING USED IN THIS CODE:
  Elements 0-4 is the location of Intake, Intermediate point, Ring front, Ring
  middle, Ring back Element 5 is the location of Place Goal

  */

  float MARGIN = 50;
  float baseSpeed = 30;
  float fourStart = mainBot.fourBarFake.position(degrees);
  float chainStart = mainBot.chainBarFake.position(degrees);

  while (true) {
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(0, 0);
    // Controller1.Screen.print("t %f %f %i %i", angles[targetIndex][0], angles[targetIndex][1], targetIndex, arrived ? 1 : 0);
    Controller1.Screen.print("%d %d %d", targetIndex, goingRight? 1 : 0, arrived ? 1 : 0);

    // angle for 4 is -1000 736
    // angle for intermediate -1323 1362
    // angle for 6 is -393 969
    // angle for 3 is -1266 497
    // angle for 2 is -942 334
    // which is 138    334
    // angle for 5 is -1505 -307

    bool isrot1 = mainBot.fourBarFake.rotateTo(angles[targetIndex][0], degrees, baseSpeed, velocityUnits::pct, false);
    bool isrot2 = mainBot.chainBarFake.rotateTo(angles[targetIndex][1], degrees, baseSpeed
    *fabs((chainStart - angles[targetIndex][1])/(fourStart - angles[targetIndex][0])), velocityUnits::pct, false);
    // rotateTo returns true if finished rotating
    // velocity control  

    int delta1 = fabs(mainBot.fourBarFake.rotation(degrees) - angles[targetIndex][0]);
    int delta2 = fabs(mainBot.fourBarFake.rotation(degrees) - angles[targetIndex][0]);
    arrived = delta1 < MARGIN && delta2 < MARGIN;

    //arrived = isrot1 && isrot2; // If true, motors are still moving to destination. If false, motors have arrived
    // Theoretically, if the motors have arrived, rotateTo should still lock the motors in place (?)

    if (arrived && targetIndex == 1) { // If at intermediate point, special case. Arm keeps moving as target node is set to next node
      targetIndex += (goingRight ? 1 : -1); // if going right, keep going right. If going left, keep going left.
      fourStart = mainBot.fourBarFake.position(degrees);
      chainStart = mainBot.chainBarFake.position(degrees);
      arrived = false; // obviously, with retargetting, we haven't arrived
    }

    if (arrived) { // Buttons only responsive if arm is not moving
      if (Controller1.ButtonRight.pressing()) {
        if (!isPressed) {
          goingRight = true;
          if (isPlaceGoal) { // any button from placing goal sends back ring middle
            targetIndex = 3;
            fourStart = mainBot.fourBarFake.position(degrees);
            chainStart = mainBot.chainBarFake.position(degrees);
            isPlaceGoal = false;
          } else {
            if (targetIndex < 4) targetIndex++;
            fourStart = mainBot.fourBarFake.position(degrees);
            chainStart = mainBot.chainBarFake.position(degrees);
          }
        }
        isPressed = true;
      } else if (Controller1.ButtonLeft.pressing()) {
        if (!isPressed) {
          goingRight = false;
          if (isPlaceGoal) { // any button from placing goal sends back ring middle
            targetIndex = 3;
            fourStart = mainBot.fourBarFake.position(degrees);
            chainStart = mainBot.chainBarFake.position(degrees);
            isPlaceGoal = false;
          } else {
            if (!isPressed && targetIndex > 0) targetIndex--;
          }
        }
        isPressed = true;
      } else if (Controller1.ButtonA.pressing()) {
        if (!isPressed && targetIndex == 3) { // Can only go to Place Goal if currently Ring middle
          // Alternatively, could be possible to press A from any location and still go to Ring middle?
          targetIndex = 5;
          fourStart = mainBot.fourBarFake.position(degrees);
          chainStart = mainBot.chainBarFake.position(degrees);
          isPlaceGoal = true;
        } else if (!isPressed && targetIndex == 5) {
          targetIndex = 3;
          fourStart = mainBot.fourBarFake.position(degrees);
          chainStart = mainBot.chainBarFake.position(degrees);
          isPlaceGoal = false;
        }
        isPressed = true;
      } else {
        isPressed = false;
      }
    }

    wait(100, msec);
  }
}

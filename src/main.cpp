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
  // Competition.drivercontrol(userControl);


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
    // Debug output
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(0, 0);
    // Controller1.Screen.print("t %f %f %i %i", angles[targetIndex][0], angles[targetIndex][1], targetIndex, arrived ? 1 : 0);
    Controller1.Screen.print("%d %d %d", targetIndex, arrived ? 1 : 0);

    // Execute motor rotation towards target!
    mainBot.fourBarFake.rotateTo(angles[targetIndex][0], degrees, baseSpeed, velocityUnits::pct, false);
    mainBot.chainBarFake.rotateTo(angles[targetIndex][1], degrees, baseSpeed
    *fabs((chainStart - angles[targetIndex][1])/(fourStart - angles[targetIndex][0])), velocityUnits::pct, false);

    // Calculate whether motor has arrived to intended target within some margin of error
    int delta1 = fabs(mainBot.fourBarFake.rotation(degrees) - angles[targetIndex][0]);
    int delta2 = fabs(mainBot.fourBarFake.rotation(degrees) - angles[targetIndex][0]);
    arrived = delta1 < MARGIN && delta2 < MARGIN;

    // Code runs whenever arm reaches a node.
    if (arrived) { 
      if (targetIndex == finalIndex) { // Buttons only responsive if arm is not moving, and arm has rested in final destination
        if (!isPressed && Controller1.ButtonDown.pressing()) {
            isPressed = true;
            finalIndex = 0;
        } else if (!isPressed && Controller1.ButtonY.pressing()) {
            isPressed = true;
            finalIndex = 2;
        } else if (!isPressed && Controller1.ButtonA.pressing()) {
            isPressed = true;
            finalIndex = 3;
        } else if (!isPressed && Controller1.ButtonX.pressing()) {
            isPressed = true;
            finalIndex = 4;
        } else if (!isPressed && Controller1.ButtonB.pressing()) {
            isPressed = true;
            finalIndex = 5;
        } else {
          isPressed = false;
        }
      }


      /*
      Since arm not currently moving, targetIndex is current location. If not equal to final location, it means
      button has been just pressed, final location has been set, and we now need to update targetIndex
      (if button pressed is already where arm is, condition will be false)
      */
      if (targetIndex != finalIndex) { 

        // A bit of hardcoding to find next target required. Refer to graph on discord.

        if (targetIndex == 0 && finalIndex > 0) targetIndex = 1; // 0 -> 1 -> anything (always goes through intermediate point)

        else if (targetIndex == 1) { // starting at intermediate point

          if (finalIndex == 5) targetIndex = 3; // Must go 1 -> 3 -> 5;
          else targetIndex = finalIndex; // For any other point, 1 -> x is fine

        } else if (targetIndex == 2 || targetIndex == 3 || targetIndex == 4) {

          if (finalIndex == 0) targetIndex = 1; // For example, 3 -> 1 -> 0

          else if (finalIndex == 5) {

            if (targetIndex == 3) targetIndex = 5; // 2 -> 3 -> 5
            else targetIndex = 3; // 3 -> 5

          } else { // This means finalIndex is 1,2,3, or 4. Just go directly to it
            targetIndex = finalIndex;
          }

        } else targetIndex = 3; // Runs if currently at 5. Can only go 5 -> 3


        // Store starting location of arm motors for purposes of velocity calculation. 
        // We must do this every time we change our target index, and arm is about to move to a new node
        fourStart = mainBot.fourBarFake.position(degrees);
        chainStart = mainBot.chainBarFake.position(degrees);

      }
      
    }

    wait(100, msec);
  }
}
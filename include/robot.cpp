#include "robot.h"
#include <math.h>

// Motor ports Left: 1R, 2F, 3F,  20T Right: 12R, 11F, 13F
// gear ratio is 60/36
Robot::Robot(controller* c) : leftMotorA(0), leftMotorB(0), leftMotorC(0), leftMotorD(0), leftMotorE(0), rightMotorA(0), rightMotorB(0), 
  rightMotorC(0), rightMotorD(0), rightMotorE(0), fourBarLeft(0), fourBarRight(0), chainBarLeft(0), chainBarRight(0), claw(0), frontCamera(0), 
  backCamera(0) {
  leftMotorA = motor(PORT1, ratio18_1, true); 
  leftMotorB = motor(PORT2, ratio18_1, true);
  leftMotorC = motor(PORT3, ratio18_1, true);
  leftMotorD = motor(PORT4, ratio18_1, true);
  leftMotorE = motor(PORT5, ratio18_1, true);
  leftDrive = motor_group(leftMotorA, leftMotorB, leftMotorC, leftMotorD, leftMotorE);

  rightMotorA = motor(PORT11, ratio18_1, false);
  rightMotorB = motor(PORT12, ratio18_1, false);
  rightMotorC = motor(PORT13, ratio18_1, false);
  rightMotorD = motor(PORT14, ratio18_1, false);
  rightMotorE = motor(PORT15, ratio18_1, false);
  rightDrive = motor_group(rightMotorA, rightMotorB, rightMotorC, rightMotorD, rightMotorE);

  fourBarLeft = motor(PORT20, ratio18_1, false);
  fourBarRight = motor(PORT17, ratio18_1, true);
  chainBarLeft = motor(PORT10, ratio18_1, false);
  chainBarRight = motor(PORT18, ratio18_1, true);
  claw = motor(PORT19, ratio18_1, false);

  YELLOW_SIG = new vision::signature(1, 1897, 2275, 2086, -3439, -3007, -3223, 11, 0);
  RED_SIG = new vision::signature (1, 6351, 10581, 8466, -1267, -537, -902, 3.300, 0);
  BLUE_SIG = new vision::signature (1, -2951, -1513, -2232, 6753, 10255, 8504, 2.500, 0);

  driveType = ARCADE;
  robotController = c; 
  frontCamera = vision(PORT9, 50, *YELLOW_SIG);
  backCamera = vision(PORT8, 50, *YELLOW_SIG);

  fourBarLeft.setBrake(hold);
  fourBarRight.setBrake(hold);
  chainBarLeft.setBrake(hold);
  chainBarRight.setBrake(hold);
  claw.setBrake(hold);
}

void Robot::driveTeleop() {
  float leftVert = (float) robotController->Axis3.position();
  float rightVert = (float) robotController->Axis2.position();
  float rightHoriz = (pow((float) robotController->Axis1.position()/100.0, 3)*100.0);

  if(driveType == ARCADE) {
    float left;
    float right;
    if(rightHoriz < 0) {
      left = leftVert + rightHoriz;
      right = leftVert - rightHoriz;
    } else {
      left = leftVert + rightHoriz;
      right = leftVert - rightHoriz;
    }

    setLeftVelocity(forward, left/fabs(left)*fmin(fabs(left), 100));
    setRightVelocity(forward, right/fabs(right)*fmin(fabs(right), 100));      
  }
}

void Robot::initArmAndClaw() {
  // Reset position of motors
  chainBarLeft.resetPosition();
  fourBarLeft.resetPosition();
  chainBarRight.resetPosition();
  fourBarRight.resetPosition();
  claw.resetPosition();

  isPressed = false; // Button presses register only at the first frame pressed. Also disallows concurrent presses from different buttons.
  arrived = true;

  finalIndex = 2; // The immediate default destination from the starting point is to Ring Front (index 2)
  targetIndex = finalIndex;

  // Store starting location of arm motors for purposes of velocity calculation
  fourStart = fourBarLeft.position(degrees);
  chainStart = chainBarLeft.position(degrees);
}

// Run every tick. Call setArmDestination() before this
// Return true when arm has reached set destination
bool Robot::armMovement(bool isTeleop, float BASE_SPEED) {
  // Code runs whenever arm reaches a node.
  if (arrived) { 
    // Getting inputs only work if in teleop mode. For auton, finalIndex will be set by function calls
    if (isTeleop && targetIndex == finalIndex) { // Buttons only responsive if arm is not moving, and arm has rested in final destination
      if (!isPressed && Robot::robotController->ButtonDown.pressing()) {
          isPressed = true;
          finalIndex = 0;
      } else if (!isPressed && Robot::robotController->ButtonY.pressing()) {
          isPressed = true;
          finalIndex = 2;
      } else if (!isPressed && Robot::robotController->ButtonA.pressing()) {
          isPressed = true;
          finalIndex = 3;
      } else if (!isPressed && Robot::robotController->ButtonX.pressing()) {
          isPressed = true;
          finalIndex = 4;
      } else if (!isPressed && Robot::robotController->ButtonB.pressing()) {
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

      if (targetIndex == 0 && finalIndex > 0) targetIndex = 6; // 0 -> 6 -> 1 -> anything (always goes through intermediate point)
      else if (targetIndex == 6) targetIndex = (finalIndex == 0 ? 0 : 1);
      else if (targetIndex == 1) { // starting at intermediate point
        if (finalIndex == 5) targetIndex = 3; // Must go 1 -> 3 -> 5;
        else if (finalIndex == 0) targetIndex = 6;
        else targetIndex = finalIndex; // For any other point, 1 -> x is fine
      }
      else if (targetIndex == 2 || targetIndex == 3 || targetIndex == 4) {
        if (finalIndex == 0) targetIndex = 1; // For example, 3 -> 1 -> 0
        else if (finalIndex == 5) {
          if (targetIndex == 3) targetIndex = 5; // 2 -> 3 -> 5
          else targetIndex = 3; // 3 -> 5
        } else { // This means finalIndex is 1,2,3, or 4. Just go directly to it
          targetIndex = finalIndex;
        }
      }
      else if (targetIndex == 5 && finalIndex == 0) targetIndex = 1;
      else targetIndex = 3; // Runs if currently at 5. Can only go 5 -> 3

      // Store starting location of arm motors for purposes of velocity calculation. 
      // We must do this every time we change our target index, and arm is about to move to a new node
      fourStart = fourBarLeft.position(degrees);
      chainStart = chainBarLeft.position(degrees);
    }
  }

  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("%d %d %d", targetIndex, finalIndex, arrived ? 1 : 0);

  float MARGIN = 10; // margin of error for if robot arm is in vicinity of target node
  //float BASE_SPEED = 30; // Base speed of arm

  // Execute motor rotation towards target!
  int chainBarVelocity = BASE_SPEED * fabs((chainStart - angles[targetIndex][1])/(fourStart - angles[targetIndex][0]));
  fourBarLeft.rotateTo(angles[targetIndex][0], degrees, BASE_SPEED, velocityUnits::pct, false);
  fourBarRight.rotateTo(angles[targetIndex][0], degrees, BASE_SPEED, velocityUnits::pct, false);
  chainBarLeft.rotateTo(angles[targetIndex][1], degrees, chainBarVelocity , velocityUnits::pct, false);
  chainBarRight.rotateTo(angles[targetIndex][1], degrees, chainBarVelocity , velocityUnits::pct, false);

  // Calculate whether motor has arrived to intended target within some margin of error
  int delta1 = fabs(fourBarLeft.rotation(degrees) - angles[targetIndex][0]);
  int delta2 = fabs(chainBarLeft.rotation(degrees) - angles[targetIndex][1]);
  arrived = delta1 < MARGIN && delta2 < MARGIN;

  return arrived && targetIndex == finalIndex;
}

void Robot::goalClamp() {
  if (Robot::robotController->ButtonL1.pressing()) {
    time_t now = std::time(nullptr);
    if(now - lastLeftPress > 0.5) {
      frontGoal.set(!frontGoal.value());
      lastLeftPress = now;
    }
  }
  if (Robot::robotController->ButtonR1.pressing()) {
    time_t now = std::time(nullptr);
    if(now - lastRightPress > 0.5) {
      backGoal.set(!backGoal.value());
      lastRightPress = now;
    }
  }
}

void Robot::clawMovement() {
  if (Robot::robotController->ButtonUp.pressing()) {
    time_t now = std::time(nullptr);
    if(now - lastClawPress > 0.5) {
      claw.rotateTo(isClawOpen? 1000 : MAX_CLAW, deg);
      isClawOpen = !isClawOpen;
      lastClawPress = now;
    }
  }
}

void Robot::setFrontClamp(bool intaking) {
  frontGoal.set(intaking);
}

void Robot::setBackClamp(bool intaking) {
  backGoal.set(intaking);
}

// Run every tick
void Robot::teleop() {
  driveTeleop();
  // armMovement(true, 50);
  clawMovement();
  goalClamp();
  wait(50, msec);
}

// Non-blocking, no-action method that updates final destination. Call armMovement() afterwards
void Robot::setArmDestination(int pos) {
  arrived = true;
  finalIndex = pos;
}

// Blocking method to move arm to location
void Robot::moveArmToPosition(int pos, float BASE_SPEED) {

  setArmDestination(pos);

  while (true) {
    if (armMovement(false, BASE_SPEED)) {
      break;
    }
    wait(100,msec);
  }
}

// dist in inches
float Robot::distanceToDegrees(float dist) {
  return dist * 360 / 2 / M_PI / (4 / 2) * 15 / 14; // 4 in diameter wheels
}

/*

// Move robot until the ultrasound reaches a certain distance, with the robot oriented so that it is parallel to the wall
// distance from wall in inches
// Keep moving forward until distToWall is <= wallDist, and the orientation is 0 within some margin
void Robot::goUltrasoundDistance(float wallDist) {

  float direction = 1000;
  float distToWall = 1000;

  float TURN_SCALAR = 1;
  float FORWARD_SCALAR = 1;

  float TURN_MARGIN = 1;
  float DIST_MARGIN = 1;

  while (fabs(direction) > TURN_MARGIN && distToWall > wallDist + DIST_MARGIN) {

    float right = rightDistSensor.distance(inches);
    float left = leftDistSensor.distance(inches);

    // negative direction means robot is turned to the left, positive means robot is turned to the right
    direction = right - left;
    distToWall = (right + left) / 2.0; // average distance to wall in inches
    float distDelta = distToWall - wallDist; // the amount (in inches) to the wall left to go

    setLeftVelocity(forward, (0 - direction)*TURN_SCALAR + distDelta * FORWARD_SCALAR);
    setRightVelocity(forward, direction*TURN_SCALAR + distDelta * FORWARD_SCALAR);

    wait(100, msec);
  }

  leftDrive.stop();
  rightDrive.stop();

}
*/

void Robot::driveStraight(float percent, float dist) {
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();
  // currPos is the current average encoder position, travelDist is the total encoder distance to be traversed, 
  // targetDist is the target encoder position, and currLeft/Right are the current left and right encoder positions
  float currLeft = leftMotorA.position(degrees);
  float currRight = rightMotorA.position(degrees);
  float currPos = (currLeft + currRight) / 2;
  float travelDist = distanceToDegrees(dist);
  float targetDist = currPos + travelDist;
  
  while (currPos < targetDist) {
    // setLeftVelocity(dist > 0 ? forward : reverse, 5 + (percent - 5) * ((targetDist - currPos) / travelDist));
    // setRightVelocity(dist > 0 ? forward : reverse, 5 + (percent - 5) * ((targetDist - currPos) / travelDist) - ((currRight - currLeft) / travelDist * 10));
    setLeftVelocity(dist > 0 ? forward : reverse, percent);
    setRightVelocity(dist > 0 ? forward : reverse, percent);
    currLeft = leftMotorA.position(degrees);
    currRight = rightMotorA.position(degrees);
    currPos = (currLeft + currRight) / 2;
  }
  leftDrive.stop();
  rightDrive.stop();
}

void Robot::driveTimed(float percent, float driveTime) {
  int milliseconds = vex::timer::system();
  while (vex::timer::system() < milliseconds + driveTime) {
    setLeftVelocity(forward, percent);
    setRightVelocity(forward, percent);
  }
  leftDrive.stop();
  rightDrive.stop();
}

//12.375 in wheelbase
void Robot::turnToAngle(float percent, float turnAngle, bool PID, directionType direction) {

  int targetDist = getTurnAngle(turnAngle);
  
  while (true) {
    if(turnToAngleNonblocking(percent, targetDist, PID, direction)) break;
    wait(100, msec);
  }
  stopLeft();
  stopRight();
}


int Robot::getTurnAngle(float turnAngle) {
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();

  // currPos is the current average encoder position, travelDist is the total encoder distance to be traversed, 
  // and targetDist is the target encoder position
  float targetDist = fabs(distanceToDegrees(turnAngle / 360 * 2 * M_PI * (15.125 / 2)));

  return targetDist;
}

// Call this method every tick. Must reset encoders of left and right motor A
// Return true if execution completed
bool Robot::turnToAngleNonblocking(float percent, float targetDist, bool PID, directionType direction) {
  float currPos = (fabs(leftMotorA.position(degrees)) + fabs(rightMotorA.position(degrees))) / 2;

  if (currPos < targetDist) {
    setLeftVelocity(direction, 5 + (percent - 5) * (PID? ((targetDist - currPos) / targetDist) : 1));
    setRightVelocity(direction == forward ? reverse : forward, 5 + (percent - 5) * (PID? ((targetDist - currPos) / targetDist) : 1));
    return false;
  } else {
    return true;
  }
}

// delta ranges from -100 (hard left) and 100 (hard right). 0 is straight
void Robot::driveCurved(directionType d, float dist, int delta) {
  int baseSpeed = 100-abs(delta)/2.0;
  int velLeft = baseSpeed + delta/2.0;
  int velRight = baseSpeed - delta/2.0;

  leftMotorA.resetPosition();
  rightMotorA.resetPosition();

  float currPos = 0.0;
  float targetPos = distanceToDegrees(dist);

  while (currPos < targetPos) {

    setLeftVelocity(d, velLeft);
    setRightVelocity(d, velRight);

    float currLeft = leftMotorA.position(degrees);
    float currRight = rightMotorA.position(degrees);
    currPos = fabs((currLeft + currRight) / 2.0);
    Robot::robotController->Screen.clearScreen();
    Robot::robotController->Screen.print(currPos);

    wait(100, msec);
  }
  // stopLeft();
  // stopRight();
}

float CENTER_X = 157.0;

bool inBounds(int x, int y,int leftBound, int rightBound, int bottomBound, int topBound) {
  return (x >= leftBound && x <= rightBound && y <= bottomBound && y >= topBound);
}

void Robot::goForwardVision(bool back, float speed, int forwardDistance, float pMod, int color) {
  vision *camera = back? &backCamera : &frontCamera;
  int brightness = color == 0? 13 : (color == 1? 22 : 40);
  camera->setBrightness(brightness);
  
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();

  float totalDist = distanceToDegrees(forwardDistance);
  float dist = 0;
  float baseSpeed = speed + pMod > 100? 100 - pMod : speed;

  while(dist < totalDist || forwardDistance == -1) {
    camera->takeSnapshot(color == 0? *YELLOW_SIG : (color == 1? *RED_SIG : *BLUE_SIG));

    float mod = camera->largestObject.exists? (CENTER_X-camera->largestObject.centerX)/CENTER_X*pMod : 0;
    
    if(camera->largestObject.exists) {
      setLeftVelocity(back? reverse : forward, baseSpeed-mod*(back?-1:1));
      setRightVelocity(back? reverse : forward, baseSpeed+mod*(back?-1:1));
    }

    wait(100, msec);
    dist = fabs((leftMotorA.position(degrees) + rightMotorA.position(degrees)) / 2.0);
  }
  stopLeft();
  stopRight();
}

void Robot::turnAndAlignVision(bool clockwise, int color) {
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();

  while(true) {
    // If completed, exit
    if(turnAndAlignVisionNonblocking(clockwise, color)) {
      return;
    }
    wait(100, msec);
  }

  stopLeft();
  stopRight();
}

//Brightness: 13 for yellow, 22 for red, 40 for blue
bool Robot::turnAndAlignVisionNonblocking(bool clockwise, int color) {
  // hopefully this is a constant-time call, if not will have to refactor
  int brightness = color == 0? 13 : (color == 1? 22 : 40);
  frontCamera.setBrightness(brightness);

  frontCamera.takeSnapshot(color == 0? *YELLOW_SIG : (color == 1? *RED_SIG : *BLUE_SIG));

  float mod = frontCamera.largestObject.exists? (CENTER_X-frontCamera.largestObject.centerX)/CENTER_X : (clockwise? 1 : -1);
  if (fabs(mod) < 0.05) {
    stopLeft();
    stopRight();
    return true;
  }
  
  float speed = 15 * (mod > 0? 1 : -1);// + 15 * cbrt(mod);
  setLeftVelocity(reverse, speed);
  setRightVelocity(forward, speed);

  return false;
}

void Robot::blindAndVisionTurn(float blindAngle, int color) {
 // Concurrently raise arm and turn robot (first blind then vision) concurrently, so use non-blocking method calls
  bool armFinished = false;
  bool turnFinished = true;
  bool blindTurnFinished = false;

  setArmDestination(2);
  int targetDist = getTurnAngle(blindAngle);

  while (true) {
    if(!armFinished) {
      armFinished = armMovement(false, 100);
    }

    if (!blindTurnFinished) {
      blindTurnFinished = turnToAngleNonblocking(100, targetDist, false, reverse);
    } else if(!turnFinished) {
      turnFinished = turnAndAlignVisionNonblocking(false, color);
    } else {
      stopLeft();
      stopRight();
    }
   
    if (armFinished && turnFinished) break;
    wait(100,msec);
  }
}


void Robot::openClaw() {
  claw.rotateTo(MAX_CLAW, deg);
  isClawOpen = true;
}

void Robot::closeClaw() {
  claw.rotateTo(1000, deg);
  isClawOpen = false;
}

void Robot::setLeftVelocity(directionType d, double percent) {
  leftMotorA.spin(d, percent, pct);
  leftMotorB.spin(d, percent, pct);
  leftMotorC.spin(d, percent, pct);
  leftMotorD.spin(d, percent, pct);
  leftMotorE.spin(d, percent, pct);
}

void Robot::setRightVelocity(directionType d, double percent) {
  rightMotorA.spin(d, percent, pct);
  rightMotorB.spin(d, percent, pct);
  rightMotorC.spin(d, percent, pct);
  rightMotorD.spin(d, percent, pct);
  rightMotorE.spin(d, percent, pct);
}

void Robot::stopLeft() {
  leftMotorA.stop();
  leftMotorB.stop();
  leftMotorC.stop();
  leftMotorD.stop();
  leftMotorE.stop();
}

void Robot::stopRight() {
  rightMotorA.stop();
  rightMotorB.stop();
  rightMotorC.stop();
  rightMotorD.stop();
  rightMotorE.stop();
}
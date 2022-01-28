#include "robot.h"
#include <math.h>

// Motor ports Left: 1R, 2F, 3F,  20T Right: 12R, 11F, 13F
// gear ratio is 60/36
Robot::Robot(controller* c, bool _isSkills) : leftMotorA(0), leftMotorB(0), leftMotorC(0), leftMotorD(0), leftMotorE(0), rightMotorA(0), rightMotorB(0), 
  rightMotorC(0), rightMotorD(0), rightMotorE(0), fourBarLeft(0), fourBarRight(0), chainBarLeft(0), chainBarRight(0), claw(0), frontCamera(0), 
  backCamera(0) {

  isSkills = _isSkills;

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

  YELLOW_SIG = new vision::signature (1, 1897, 2275, 2086, -3439, -3007, -3223, 11, 0);
  RED_SIG = new vision::signature (1, 6351, 10581, 8466, -1267, -537, -902, 3.300, 0);
  BLUE_SIG = new vision::signature (1, -2505, -1535, -2020, 6483, 9831, 8157, 2.500, 0);

  driveType = ARCADE;
  robotController = c; 
  frontCamera = vision(PORT9, 50, *YELLOW_SIG);
  backCamera = vision(PORT8, 50, *YELLOW_SIG);

  fourBarLeft.setBrake(hold);
  fourBarRight.setBrake(hold);
  chainBarLeft.setBrake(hold);
  chainBarRight.setBrake(hold);
  claw.setBrake(hold);

  // Skills place goal position
  if (isSkills) {
    angles[5][0] = 50.8;
    angles[5][1] = -92.8;
  }
}

void Robot::driveTeleop() {
  float leftVert = (float) robotController->Axis3.position();
  float rightVert = (float) robotController->Axis2.position();
  float rightHoriz = (pow((float) robotController->Axis1.position()/100.0, 3)*100.0);

  if(driveType == ARCADE) {
    float left;
    float right;
    if(rightHoriz < 0) {
      left = leftVert + pow(rightHoriz, 3);
      right = leftVert - pow(rightHoriz, 3);
    } else {
      left = leftVert + pow(rightHoriz, 3);
      right = leftVert - pow(rightHoriz, 3);
    }

    setLeftVelocity(forward, left/fabs(left)*fmin(fabs(left), 100));
    setRightVelocity(forward, right/fabs(right)*fmin(fabs(right), 100));      
  } else {
    setLeftVelocity(forward, leftVert * 100);
    setRightVelocity(forward,  rightVert * 100);      
  }
}

void Robot::initArmAndClaw() {
  // chainBarLeft.setBrake(coast);
  // chainBarRight.setBrake(coast);
  // fourBarLeft.spin(reverse, 10, pct);
  // fourBarRight.spin(reverse, 10, pct);
  // wait(1, sec);
  // chainBarLeft.setBrake(hold);
  // chainBarRight.setBrake(hold);

  // Reset position of motors
  chainBarLeft.resetPosition();
  fourBarLeft.resetPosition();
  chainBarRight.resetPosition();
  fourBarRight.resetPosition();
  claw.resetPosition();

  isPressed = false; // Button presses register only at the first frame pressed. Also disallows concurrent presses from different buttons.
  arrived = true;

  finalIndex = RING_FRONT; // The immediate default destination from the starting point is to Ring Front (index 2)
  prevIndex = RING_FRONT;
  targetIndex = finalIndex;

  // Store starting location of arm motors for purposes of velocity calculation
  fourStart = fourBarLeft.position(degrees);
  chainStart = chainBarLeft.position(degrees);
}

// Run every tick. Call setArmDestination() before this
// Return true when arm has reached set destination
bool Robot::armMovement(bool isTeleop, float BASE_SPEED, bool isSkills) {


  bool isTimeout = vex::timer::system() - armTimeout > ARM_TIMEOUT_MS;
  if (isTimeout) {
    finalIndex = prevIndex;
  }

  // Code runs whenever arm reaches a node.
  if (arrived || isTimeout) { 

    // When arrived, prevIndex is finalIndex. But once a button is pressed, prevIndex stays the same number while finalIndex changes
    armTimeout = vex::timer::system();

    // Getting inputs only work if in teleop mode. For auton, finalIndex will be set by function calls
    if (!isTimeout && isTeleop && targetIndex == finalIndex) { // Buttons only responsive if arm is not moving, and arm has rested in final destination
      if (!isPressed && Robot::robotController->ButtonDown.pressing()) {
          isPressed = true;
          prevIndex = finalIndex;
          finalIndex = INTAKING;
      } else if (!isPressed && Robot::robotController->ButtonY.pressing()) {
          isPressed = true;
          prevIndex = finalIndex;
          finalIndex = RING_FRONT;
      } else if (!isPressed && Robot::robotController->ButtonA.pressing()) {
          isPressed = true;
          prevIndex = finalIndex;
          finalIndex = ABOVE_MIDDLE;
      } else if (!isPressed && Robot::robotController->ButtonX.pressing()) {
          isPressed = true;
          prevIndex = finalIndex;
          finalIndex = RING_BACK;
      } else if (!isPressed && Robot::robotController->ButtonB.pressing()) {
          isPressed = true;
          prevIndex = finalIndex;
          finalIndex = PLACE_GOAL;
      } else if (!isPressed && Robot::robotController->ButtonRight.pressing()) {
          isPressed = true;
          prevIndex = finalIndex;
          finalIndex = PLATFORM_LEVEL;
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


      // INTAKING = 0, INTER_INNER = 1, RING_FRONT = 2, ABOVE_MIDDLE = 3, RING_BACK = 4, PLACE_GOAL = 5, INTER_FRONT = 6, PLATFORM_LEVEL = 7

      // If skills, starting at position 0 or 7, and going to 2/3/4, go directly there instead of through 1 and 6. If going to 5, go through 3 to 5
      if (isSkills && targetIndex == INTAKING && finalIndex >= RING_FRONT && finalIndex <= PLACE_GOAL) {
        targetIndex = PLATFORM_LEVEL;
      }
      else if (isSkills && targetIndex == PLATFORM_LEVEL && finalIndex >= RING_FRONT && finalIndex <= PLACE_GOAL) {
        targetIndex = (finalIndex == PLACE_GOAL) ? ABOVE_MIDDLE : finalIndex;
      } 
      else if (isSkills && (targetIndex >= RING_FRONT && targetIndex <= PLACE_GOAL) && (finalIndex == INTAKING || finalIndex == PLATFORM_LEVEL)) {
        targetIndex = (targetIndex == PLACE_GOAL) ? ABOVE_MIDDLE : PLATFORM_LEVEL;
      }
      else if (targetIndex == INTAKING && finalIndex > INTAKING) targetIndex = PLATFORM_LEVEL; // 0 -> 7 -> 6 -> 1 -> anything (always goes through intermediate point)
      else if (targetIndex == INTER_FRONT) targetIndex = (finalIndex == INTAKING ? PLATFORM_LEVEL : INTER_INNER);
      else if (targetIndex == PLATFORM_LEVEL) targetIndex = (finalIndex == INTAKING ? INTAKING : INTER_FRONT);
      else if (targetIndex == INTER_INNER) { // starting at intermediate point
        if (finalIndex == PLACE_GOAL) targetIndex = ABOVE_MIDDLE; // Must go 1 -> 3 -> 5;
        else if (finalIndex == INTAKING) targetIndex = INTER_FRONT;
        else targetIndex = finalIndex; // For any other point, 1 -> x is fine
      }
      else if (targetIndex == RING_FRONT || targetIndex == ABOVE_MIDDLE || targetIndex == RING_BACK) {
        if (finalIndex == INTAKING) targetIndex = INTER_INNER; // For example, 3 -> 1 -> 0
        else if (finalIndex == PLACE_GOAL) {
          if (targetIndex == ABOVE_MIDDLE) targetIndex = PLACE_GOAL; // 2 -> 3 -> 5
          else targetIndex = ABOVE_MIDDLE; // 3 -> 5
        } else { // This means finalIndex is 1,2,3, or 4. Just go directly to it
          targetIndex = finalIndex;
        }
      }
      else if (targetIndex == 5 && finalIndex == INTAKING) targetIndex = INTER_INNER;
      else targetIndex = ABOVE_MIDDLE; // Runs if currently at 5. Can only go 5 -> 3

      // Store starting location of arm motors for purposes of velocity calculation. 
      // We must do this every time we change our target index, and arm is about to move to a new node
      fourStart = fourBarLeft.position(degrees);
      chainStart = chainBarLeft.position(degrees);
    }
  }

  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("%d %d %d %d %d", targetIndex, finalIndex, prevIndex, arrived ? 1 : 0, vex::timer::system() - armTimeout);

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
    if(!frontWasPressed) {
      frontGoal.set(!frontGoal.value());
    }
    frontWasPressed = true;
  } else {
    frontWasPressed = false;
  }

  if (Robot::robotController->ButtonR1.pressing()) {
    if(!backWasPressed) {
      backGoal.set(!backGoal.value());
    }
    backWasPressed = true;
  } else {
    backWasPressed = false;
  }
}

void Robot::clawMovement() {
  if (Robot::robotController->ButtonUp.pressing()) {
    if(!clawWasPressed) {
      claw.rotateTo(isClawOpen? 500 : MAX_CLAW, deg, 100, velocityUnits::pct, false);
      isClawOpen = !isClawOpen;
    }
    clawWasPressed = true;
  } else {
    clawWasPressed = false;
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
  armMovement(true, 100, isSkills);
  clawMovement();
  goalClamp();
  wait(20, msec);
}

// Non-blocking, no-action method that updates final destination. Call armMovement() afterwards
void Robot::setArmDestination(Arm pos) {
  arrived = true;
  finalIndex = pos;
}

// Blocking method to move arm to location
void Robot::moveArmToPosition(Arm pos, float BASE_SPEED) {

  setArmDestination(pos);

  while (true) {
    if (armMovement(false, BASE_SPEED)) {
      break;
    }
    wait(20,msec);
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

    wait(20, msec);
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
  float currLeft = 0;
  float currRight = 0;
  float currPos = (currLeft + currRight) / 2;
  float travelDist = fabs(distanceToDegrees(dist));
  
  while (currPos < travelDist) {
    // setLeftVelocity(dist > 0 ? forward : reverse, 5 + (percent - 5) * ((travelDist - currPos) / travelDist));
    // setRightVelocity(dist > 0 ? forward : reverse, 5 + (percent - 5) * ((travelDist - currPos) / travelDist) - ((currRight - currLeft) / travelDist * 10));
    setLeftVelocity(dist > 0 ? forward : reverse, percent);
    setRightVelocity(dist > 0 ? forward : reverse, percent);
    currLeft = leftMotorA.position(degrees);
    currRight = rightMotorA.position(degrees);
    currPos = fabs((currLeft + currRight) / 2);
  }
  stopLeft();
  stopRight();
}

void Robot::driveTimed(float percent, float driveTime) {
  int milliseconds = vex::timer::system();
  while (vex::timer::system() < milliseconds + driveTime) {
    setLeftVelocity(forward, percent);
    setRightVelocity(forward, percent);
  }
  stopLeft();
  stopRight();
}

//12.375 in wheelbase
void Robot::turnToAngle(float percent, float turnAngle, bool PID, directionType direction) {

  int targetDist = getTurnAngle(turnAngle);
  
  while (true) {
    if(turnToAngleNonblocking(percent, targetDist, PID, direction)) break;
    wait(20, msec);
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

    wait(20, msec);
  }
  // stopLeft();
  // stopRight();
}

float CENTER_X = 157.0;

bool inBounds(int x, int y,int leftBound, int rightBound, int bottomBound, int topBound) {
  return (x >= leftBound && x <= rightBound && y <= bottomBound && y >= topBound);
}

void Robot::goForwardVision(bool back, float speed, int forwardDistance, float pMod, int color) {
  frontCamera = vision(PORT9, 50, color == 0? *YELLOW_SIG : (color == 1? *RED_SIG : *BLUE_SIG));
  frontCamera = vision(PORT9, 50, color == 0? *YELLOW_SIG : (color == 1? *RED_SIG : *BLUE_SIG));
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

    wait(20, msec);
    dist = fabs((leftMotorA.position(degrees) + rightMotorA.position(degrees)) / 2.0);
  }
  stopLeft();
  stopRight();
}

void Robot::turnAndAlignVision(bool clockwise, int color, float modThresh) {
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();
  frontCamera = vision(PORT9, 50, color == 0? *YELLOW_SIG : (color == 1? *RED_SIG : *BLUE_SIG));

  while(true) {
    // If completed, exit
    if(turnAndAlignVisionNonblocking(clockwise, color, modThresh)) {
      return;
    }
    wait(20, msec);
  }

  stopLeft();
  stopRight();
}

//Brightness: 13 for yellow, 22 for red, 40 for blue
bool Robot::turnAndAlignVisionNonblocking(bool clockwise, int color, float modThresh) {
  // hopefully this is a constant-time call, if not will have to refactor
  int brightness = color == 0? 13 : (color == 1? 22 : 40);
  frontCamera.setBrightness(brightness);
  // frontCamera.setSignature(color == 0? *YELLOW_SIG : (color == 1? *RED_SIG : *BLUE_SIG));

  frontCamera.takeSnapshot(color == 0? *YELLOW_SIG : (color == 1? *RED_SIG : *BLUE_SIG));

  float mod = frontCamera.largestObject.exists? (CENTER_X-frontCamera.largestObject.centerX)/CENTER_X : (clockwise? -1 : 1);
  if (fabs(mod) < modThresh) {
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
  bool turnFinished = false;
  bool blindTurnFinished = false;

  setArmDestination(RING_FRONT);
  int targetDist = getTurnAngle(blindAngle);

  while (true) {
    if(!armFinished) {
      armFinished = armMovement(false, 100);
    }

    if (!blindTurnFinished) {
      blindTurnFinished = turnToAngleNonblocking(100, targetDist, false, reverse);
    } else if(!turnFinished) {
      turnFinished = turnAndAlignVisionNonblocking(false, color, 0.05);
    } else {
      stopLeft();
      stopRight();
    }
   
    if (armFinished && turnFinished) break;
    wait(20, msec);
  }
}


void Robot::openClaw() {
  claw.rotateTo(MAX_CLAW, deg, 100, velocityUnits::pct);
  isClawOpen = true;
}

void Robot::closeClaw() {
  int clawTimeout = vex::timer::system();
  while(true) {
    if (vex::timer::system() - clawTimeout > 1000) { return; }
    claw.rotateTo(500, deg, 100, velocityUnits::pct, false);
    wait(20, msec);
  }
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

void Robot::intakeOverGoal(int color) {
  turnToAngle(100, -60, false, forward);
  turnAndAlignVision(true, color, 0.1);
  goForwardVision(false, 20, 10, 40, color);
  moveArmToPosition(INTAKING, 100);
  openClaw();
  driveStraight(20, 8);
  closeClaw();
  moveArmToPosition(ABOVE_MIDDLE, 100);
  setFrontClamp(true);
  moveArmToPosition(PLACE_GOAL, 100);
  openClaw();
  moveArmToPosition(ABOVE_MIDDLE, 100);
  setFrontClamp(false);
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


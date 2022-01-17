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

  fourBarLeft = motor(PORT10, ratio18_1, true);
  fourBarRight = motor(PORT8, ratio18_1, false);
  chainBarLeft = motor(PORT19, ratio18_1, true);
  chainBarRight = motor(PORT7, ratio18_1, false);
  claw = motor(16, ratio18_1, true);

  SIG_1 = new vision::signature(1, 1757, 2979, 2368, -3019, -2351, -2685, 2.100, 0);

  driveType = ARCADE;
  robotController = c; 
  frontCamera = vision(PORT20, 50, *SIG_1);
  backCamera = vision(PORT6, 50, *SIG_1);

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
    
    // if(fabs(leftVert) < 5.0) {
    //   robotController->Screen.setCursor(1, 1);
    //   robotController->Screen.print("POINT TURN");
    //   setLeftVelocity(forward, rightHoriz/fabs(rightHoriz)*fmin(fabs(rightHoriz), 100));
    //   setRightVelocity(reverse, rightHoriz/fabs(rightHoriz)*fmin(fabs(rightHoriz), 100));
    // } else {
      setLeftVelocity(forward, left/fabs(left)*fmin(fabs(left), 100));
      setRightVelocity(forward, right/fabs(right)*fmin(fabs(right), 100));
    // }
    
  }


  // float leftJoystick = (driveType == ARCADE) ? leftVert^3 + rightHoriz^3: leftVert^3;
  // float rightJoystick = (driveType == ARCADE) ? leftVert^3 + rightHoriz^3: rightVert^3;

  // float oldLeft = 0;
  // float oldRight = 0;

  // if (fabs(leftJoystick) > 5) {
  //   float percent = (driveType == ARCADE) ? (pow((leftVert/100.00f), 3.00f) + pow((rightHoriz/100.00f), 5.00f))*100.00f : leftJoystick;
  //   setLeftVelocity(forward, percent/fabs(percent)*fmin(fabs(percent), 100)); //60 is max for now
  //   oldLeft = percent;
  // } else {
  //   stopLeft();
  // }

  // if (fabs(rightJoystick) > 5) {
  //   float percent = (driveType == ARCADE) ? (pow((leftVert/100.00f), 3.00f) - pow((rightHoriz/100.00f), 5.00f))*100.00f : rightJoystick;
  //   setRightVelocity(forward, percent/fabs(percent)*fmin(fabs(percent), 100));
  //   oldRight = percent;
  // } else {
  //   stopRight();
  // }
}

void Robot::initArm() {
  // Reset position of motors
  chainBarLeft.resetPosition();
  fourBarLeft.resetPosition();
  chainBarRight.resetPosition();
  fourBarRight.resetPosition();

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
      fourStart = fourBarLeft.position(degrees);
      chainStart = chainBarLeft.position(degrees);
    }
  }

  float MARGIN = 10; // margin of error for if robot arm is in vicinity of target node
  //float BASE_SPEED = 30; // Base speed of arm

  
  //Robot::robotController->Screen.print(targetIndex);

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

  // Debug output
  // Robot::robotController->Screen.clearScreen();
  // Robot::robotController->Screen.setCursor(0, 0);
  // Robot::robotController->Screen.print("t %d %d %d %d %d", (int)angles[targetIndex][0], (int)angles[targetIndex][1], targetIndex, delta1, delta2);

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
  goalClamp();
  // time_t now = std::time(nullptr);
  // robotController->Screen.clearScreen();
  // robotController->Screen.print(now - lastLeftPress);
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

  while (false) {
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
    setLeftVelocity(dist > 0 ? forward : reverse, 5 + (percent - 5) * ((targetDist - currPos) / travelDist));
    setRightVelocity(dist > 0 ? forward : reverse, 5 + (percent - 5) * ((targetDist - currPos) / travelDist) - ((currRight - currLeft) / travelDist * 10));
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
  //stopLeft();
  //stopRight();
}

float CENTER_X = 157.0;

bool inBounds(int x, int y,int leftBound, int rightBound, int bottomBound, int topBound) {
  return (x >= leftBound && x <= rightBound && y <= bottomBound && y >= topBound);
}

void Robot::goForwardVision(bool back, float speed, int forwardDistance) {

  vision *camera = back? &backCamera : &frontCamera;
  backCamera.setBrightness(19);
  
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();

  float totalDist = distanceToDegrees(forwardDistance);
  float dist = 0;
  float pMod = 15.0;
  float baseSpeed = speed + pMod > 100? 100 - pMod : speed;

  while(dist < totalDist || forwardDistance == -1) {
    camera->takeSnapshot(*SIG_1);

    float mod = camera->largestObject.exists? (CENTER_X-camera->largestObject.centerX)/CENTER_X*pMod : 0;
    
    if(camera->largestObject.exists) {
      setLeftVelocity(back? reverse : forward, baseSpeed-mod*(back?1:-1));
      setRightVelocity(back? reverse : forward, baseSpeed+mod*(back?1:-1));
    }

    wait(100, msec);
    dist = fabs((leftMotorA.position(degrees) + leftMotorB.position(degrees)) / 2.0);
  }

  // stopLeft();
  // stopRight();
}

void Robot::turnAndAlignVision(bool clockwise) {

  leftMotorA.resetPosition();
  rightMotorA.resetPosition();


  while(true) {
    
    // If completed, exit
    if(turnAndAlignVisionNonblocking(clockwise)) {
      return;
    }
    wait(100, msec);
  }

  stopLeft();
  stopRight();
}

bool Robot::turnAndAlignVisionNonblocking(bool clockwise) {

  // hopefully this is a constant-time call, if not will have to refactor
  frontCamera.setBrightness(13);
  float baseSpeed = 30;

  frontCamera.takeSnapshot(*SIG_1);

  float mod = frontCamera.largestObject.exists? (CENTER_X-frontCamera.largestObject.centerX)/CENTER_X : (clockwise? -1 : 1);

  if (mod < 0.1) return true;
    
  setLeftVelocity(reverse, baseSpeed*mod);
  setRightVelocity(forward, baseSpeed*mod);

  return false;

}


void Robot::openClaw() {}
void Robot::closeClaw() {}

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
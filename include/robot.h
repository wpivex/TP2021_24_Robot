#pragma once
#include "vex.h"
#include <string>
#include <fstream>
#include <vector>
#include <utility> // std::pair
#include <stdexcept> // std::runtime_error
#include <sstream> // std::stringstream
#include <string>
#include <iostream>
#include <sstream>
#include <stdlib.h>     /* atof */
#include <math.h>       /* sin */
#include <stdio.h>      /* printf, fgets */
#include <unistd.h>

using namespace vex;
brain Brain;

class Robot {
  public:
    // four drivebase motors will not be accessible for a while
    Robot(controller* c);
    motor leftMotorA;
    motor leftMotorB;
    motor leftMotorC;
    motor leftMotorD;
    motor leftMotorE;
    motor rightMotorA;
    motor rightMotorB;
    motor rightMotorC;
    motor rightMotorD;
    motor rightMotorE;

    motor_group leftDrive;
    motor_group rightDrive;

    vision backCamera;
    vision frontCamera;

    motor fourBarLeft;
    motor fourBarRight;
    motor chainBarLeft;
    motor chainBarRight;
    motor claw;

    digital_out frontGoal = digital_out(Brain.ThreeWirePort.A);
    digital_out backGoal = digital_out(Brain.ThreeWirePort.B);

    controller* robotController;

    vision::signature* YELLOW_SIG;
    vision::signature* RED_SIG;
    vision::signature* BLUE_SIG;

    enum Arm { INTAKING = 0, INTER_INNER = 1, RING_FRONT = 2, ABOVE_MIDDLE = 3, RING_BACK = 4, PLACE_GOAL = 5, INTER_FRONT = 6, PLATFORM_LEVEL = 7 };

    void driveStraight(float percent, float dist);
    void driveStraight(float percent, float dist, float accPercent);
    void driveTimed(float percent, float driveTime);
    int getTurnAngle(float turnAngle);
    void turnToAngle(float percent, float turnAngle, bool PID, directionType direction);
    bool turnToAngleNonblocking(float percent, float targetDist, bool PID, directionType direction);
    void driveCurved(directionType d, float dist, int delta);
    void goForwardVision(bool back, float speed, int forwardDistance, float pMod, int color);
    void turnAndAlignVision(bool clockwise, int brightness, float modThresh);
    bool turnAndAlignVisionNonblocking(bool clockwise, int color, float modThresh);
    void blindAndVisionTurn(float blindAngle, int color);
    float distanceToDegrees(float dist);
    void openClaw();
    void closeClaw();
    void goalClamp();
    void setFrontClamp(bool intaking);
    void setBackClamp(bool intaking);

    void userControl( void );
    void teleop( void );
    void initArmAndClaw();
    void setArmDestination(Arm pos);
    bool armMovement(bool isTeleop, float BASESPEED);
    void moveArmToPosition(Arm pos, float BASESPEED);
    void setLeftVelocity(directionType d, double percent);
    void setRightVelocity(directionType d, double percent);
    void goUltrasoundDistance(float dist);
    void stopLeft();
    void stopRight();
    void intakeOverGoal(int color);

    enum DriveType { ARCADE, TANK };
    DriveType driveType;

  private:

    // fourbar, chainbar
    double angles[8][2] = {{394, 1140}, //intaking (0)
                          {1492.4, 682}, //intermediate 1 (1) (farther into robot)
                          {1060, 428}, //ring front (2)
                          {1375.2, 563.2}, //ring middle (3)
                          {1446, -26}, //ring back (4)
                          {509.2, 140}, //place goal (5)
                          {1339.2, 1777.6}, //intermediate 2 (6) (farther out of robot)
                          {1339.2, 1777.6}}; //score goal on platform (7)

    void driveTeleop();
    void pneumaticsTeleop();
    void clawMovement();
    

    // State variables for arm teleop code
    bool isPressed;
    bool arrived;
    Arm finalIndex, targetIndex, prevIndex;
    float fourStart, chainStart;
    int armTimeout;

    float ARM_TIMEOUT_MS = 3000; // 3 seconds for timeout per motion

    // State variables for claw
    float MAX_CLAW = -520;
    bool isClawOpen = false;


    // State variables for goal clamp
    time_t lastLeftPress = std::time(nullptr);
    time_t lastRightPress = std::time(nullptr);
    time_t lastClawPress = std::time(nullptr);

    bool frontWasPressed = false;
    bool backWasPressed = false;
    bool clawWasPressed = false;

    // time_t lastLeftPress = std::time(nullptr);
    // time_t lastRightPress = std::time(nullptr);
    // time_t lastClawPress = std::time(nullptr);
};
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

class Robot {
  public:
    // four drivebase motors will not be accessible for a while
    Robot(controller* c, brain* b);
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

    pneumatics piston;


    controller* robotController;
    brain* brainn;

    vision::signature* SIG_1;

    void driveStraight(float percent, float dist);
    void driveStraight(float percent, float dist, float accPercent);
    void driveTimed(float percent, float driveTime);
    int getTurnAngle(float turnAngle);
    void turnToAngle(float percent, float turnAngle, bool PID, directionType direction);
    bool turnToAngleNonblocking(float percent, float targetDist, bool PID, directionType direction);
    void driveCurved(directionType d, float dist, int delta);
    void goForwardVision(bool back, int forwardDistance);
    void turnAndAlignVision(bool clockwise);
    bool turnAndAlignVisionNonblocking(bool clockwise);
    float distanceToDegrees(float dist);
    void openClaw();
    void closeClaw();

    void userControl( void );
    void teleop( void );
    void initArm();
    void setArmDestination(int pos);
    bool armMovement(bool isTeleop, float BASESPEED);
    void moveArmToPosition(int pos, float BASESPEED);
    void setLeftVelocity(directionType d, double percent);
    void setRightVelocity(directionType d, double percent);
    void goUltrasoundDistance(float dist);
    void stopLeft();
    void stopRight();

    enum DriveType { ARCADE, TANK };
    DriveType driveType;

  private:

    // fourbar, chainbar
    double angles[6][2] = {{422, 821}, //intaking (0)
                          {1311, 1247}, //intermediate (1)
                          {1060, 428}, //ring front (2)
                          {1271, 327}, //ring middle (3)
                          {1446, -26}, //right back (4)
                          {500, 220}}; //place goal (5)

    void driveTeleop();
    void pneumaticsTeleop();
    

    // State variables for arm teleop code
    bool isPressed;
    bool arrived;
    int finalIndex, targetIndex;
    float fourStart, chainStart;
};
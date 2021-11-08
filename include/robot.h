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
    Robot(controller* c, double **angles);
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

    vision camera;

    motor fourBarLeft;
    motor fourBarRight;
    motor chainBarLeft;
    motor chainBarRight;
    motor claw;

    controller* robotController;

    void driveStraight(float percent, float dist);
    void driveStraight(float percent, float dist, float accPercent);
    void driveTimed(float percent, float driveTime);
    void turnToAngle(float percent, float turnAngle);
    void driveCurved(directionType d, float dist, int delta);
    float distanceToDegrees(float dist);
    void openClaw();
    void closeClaw();
    void liftFourBar(float percentHeight);
    void lowerFourBar(float percentHeight);

    void userControl( void );
    void teleop( void );
    void initArm();
    void moveArmToPosition(int pos);
    void setLeftVelocity(directionType d, double percent);
    void setRightVelocity(directionType d, double percent);
    void stopLeft();
    void stopRight();

    enum DriveType { ARCADE, TANK };
    DriveType driveType;

  private:

    double** angles;

    void driveTeleop();
    void armMovement(bool isTeleop);

    // State variables for arm teleop code
    bool isPressed;
    bool arrived;
    int finalIndex, targetIndex;
    float fourStart, chainStart;



};
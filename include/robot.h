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
#include <constants.h>

using namespace vex;
brain Brain;

class Robot {
  public:
    // four drivebase motors will not be accessible for a while
    Robot(controller* c, bool isSkills);
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

    inertial gyroSensor;

    vision::signature* YELLOW_SIG;
    vision::signature* RED_SIG;
    vision::signature* BLUE_SIG;

    enum Arm { INTAKING = 0, INTER_INNER = 1, RING_FRONT = 2, ABOVE_MIDDLE = 3, RING_BACK = 4, PLACE_GOAL = 5, INTER_FRONT = 6, PLATFORM_LEVEL = 7 };


    void smartDrive(float distInches, float speed, directionType left, directionType right, int timeout, float slowDownInches, 
      float turnPercent, bool stopAfter, std::function<bool(void)> func);
    void driveTurn(float degrees, float speed, bool isClockwise, int timeout, float slowDownInches = 10, 
      bool stopAfter = true, std::function<bool(void)> func = {});
    void driveCurved(float distInches, float speed, directionType dir, int timeout, 
      float slowDownInches, float turnPercent, bool stopAfter = true, std::function<bool(void)> func = {});
    void driveStraight(float distInches, float speed, directionType dir, int timeout, 
      float slowDownInches, bool stopAfter = true, std::function<bool(void)> func = {});
    void driveStraightTimed(float speed, directionType dir, int timeMs, bool stopAfter, std::function<bool(void)> func = {});

    void goForwardVision(Goal goal, float speed, directionType dir, float maximumDistance, int timeout, 
    digital_in* limitSwitch, std::function<bool(void)> func = {});
    void alignToGoalVision(Goal goal, bool clockwise, directionType cameraDirection, int timeout);

    void driveStraightGyro(float distInches, float speed, directionType dir, int timeout, float slowDownInches, std::function<bool(void)> func);
    void turnToAngleGyro(bool clockwise, float angleDegrees, float maxSpeed, int startSlowDownDegrees,
int timeout, std::function<bool(void)> func = {});

    void goForwardVision(bool back, float speed, int forwardDistance, float pMod, int color);
    void turnAndAlignVision(bool clockwise, int brightness, float modThresh, bool returnImmediate);
    bool turnAndAlignVisionNonblocking(bool clockwise, int color, float modThresh, bool returnImmediate);
    void blindAndVisionTurn(float blindAngle, int color);
    float distanceToDegrees(float dist);
    void openClaw();
    void closeClaw();
    void goalClamp();
    void setFrontClamp(bool intaking);
    void setBackClamp(bool intaking);

    void userControl( void );
    void teleop( void );
    void setLeftVelocity(directionType d, double percent);
    void setRightVelocity(directionType d, double percent);
    void goUltrasoundDistance(float dist);
    void stopLeft();
    void stopRight();
    void intakeOverGoal(int color);

    enum DriveType { ARCADE, TANK };
    DriveType driveType;


  private:


    void driveTeleop();
    void pneumaticsTeleop();
    void clawMovement();
    

    float fourStart, chainStart;


    // State variables for claw
    float MAX_CLAW = -520;
    bool isClawOpen = false;

    bool frontWasPressed = false;
    bool backWasPressed = false;
    bool clawWasPressed = false;

    bool isSkills;
};
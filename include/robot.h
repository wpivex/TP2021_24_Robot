#pragma once
#include "ArmGraph.h"
#include "ArmGraph.cpp"
#include "Buttons.cpp"
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
#include "constants.h"
#include <stdarg.h>

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

    digital_out frontGoal = digital_out(Brain.ThreeWirePort.B);
    digital_out backGoal = digital_out(Brain.ThreeWirePort.C);

    controller* robotController;

    inertial gyroSensor;

    ArmGraph arm;
    Buttons buttons;

    enum DriveType { ONE_STICK_ARCADE, TWO_STICK_ARCADE, TANK };
    DriveType driveType;

    enum ControllerMapping {DEFAULT_MAPPING};
    ControllerMapping cMapping;
    Buttons::Button FRONT_CLAMP_TOGGLE, BACK_CLAMP_TOGGLE, CLAW_TOGGLE; 

    void setControllerMapping(ControllerMapping mapping);

    void smartDrive(float distInches, float speed, directionType left, directionType right, int timeout, float slowDownInches, 
                    float turnPercent, bool stopAfter, std::function<bool(void)> func);
    void driveTurn(float degrees, float speed, bool isClockwise, int timeout, float slowDownInches = 10, 
                    bool stopAfter = true, std::function<bool(void)> func = {});
    void driveCurved(float distInches, float speed, directionType dir, int timeout, 
                      float slowDownInches, float turnPercent, bool stopAfter = true, std::function<bool(void)> func = {});
    void driveStraight(float distInches, float speed, directionType dir, int timeout, 
                      float slowDownInches, bool stopAfter = true, std::function<bool(void)> func = {});
    void driveStraightTimed(float speed, directionType dir, int timeMs, bool stopAfter = true, std::function<bool(void)> func = {});

    void goForwardVision(Goal goal, float speed, directionType dir, float maximumDistance, int timeout, 
                        digital_in* limitSwitch, std::function<bool(void)> func = {}, float pModMult = 0.2);
    void alignToGoalVision(Goal goal, bool clockwise, directionType cameraDirection, int timeout, float maxSpeed = 20);
    void updateCamera(Goal goal);

    void driveStraightGyro(float distInches, float speed, directionType dir, int timeout, float slowDownInches,
                            std::function<bool(void)> func = {});
    void turnToAngleGyro(bool clockwise, float angleDegrees, float maxSpeed, int startSlowDownDegrees, 
                        int timeout, float tolerance = 1, std::function<bool(void)> func = {});
    void turnToUniversalAngleGyro(float universalAngleDegrees, float maxSpeed, int startSlowDownDegrees, 
                        int timeout, float tolerance = 1, std::function<bool(void)> func = {});
    void gyroTurnU(float universalAngleDegrees);
    void gyroTurn(bool clockwise, float angleDegrees);

    void callibrateGyro();

    void openClaw(bool waitForCompletion = true);
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

  private:


    void driveTeleop();
    void pneumaticsTeleop();
    void clawMovement();
    

    float fourStart, chainStart;


    // State variables for claw
    float MAX_CLAW = -256;
    bool isClawOpen = false;

    bool isSkills;
};
#pragma once
#include "ArmGraph.cpp"
#include "TrapezoidalController.cpp"
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

    pot chainBarPot = pot(Brain.ThreeWirePort.H);
    bumper fourBarBump = bumper(Brain.ThreeWirePort.A);

    digital_out frontGoal = digital_out(Brain.ThreeWirePort.B);
    digital_out backGoal = digital_out(Brain.ThreeWirePort.C);

    digital_out clawPiston = digital_out(Brain.ThreeWirePort.G);

    controller* robotController;

    inertial gyroSensor;

    ArmGraph arm;
    Buttons buttons;

    enum DriveType { ONE_STICK_ARCADE, TWO_STICK_ARCADE, TANK };
    DriveType driveType;

    enum ControllerMapping {DEFAULT_MAPPING};
    ControllerMapping cMapping;
    Buttons::Button FRONT_CLAMP_TOGGLE, BACK_CLAMP_TOGGLE, CLAW_TOGGLE, ARM_TOGGLE; 

    void setControllerMapping(ControllerMapping mapping);

    void callibrateGyro();

    void openClaw();
    void closeClaw();
    void goalClamp();
    void setFrontClamp(bool intaking);
    void setBackClamp(bool intaking);

    void userControl( void );
    void teleop( void );
    void setLeftVelocity(directionType d, double percent);
    void setRightVelocity(directionType d, double percent);
    void stopLeft();
    void stopRight();
    void setBrakeType(brakeType b);
    void setMaxArmTorque(float c);

    vision getCamera(directionType dir, Goal goal);

    // --------- NEW FUNCTIONS -----------

    void goForward(float distInches, float maxSpeed, float rampUpInches = 0, float slowDownInches = 5,
      int timeout = 5, std::function<bool(void)> func = {});

    void goForwardUniversal(float distInches, float maxSpeed, float universalAngle, float rampUpInches = 0, float slowDownInches = 5,
      int timeout = 5, std::function<bool(void)> func = {});

    void goCurve(float distInches, float maxSpeed, float turnPercent, float rampUpInches = 0, float slowDownInches = 5,
      int timeout = 5, std::function<bool(void)> func = {});

    void goVision(float distInches, float maxSpeed, Goal goal, directionType cameraDir, 
    float rampUpInches = 0, float slowDownInches = 5, int timeout = 5, bool stopAfter = true, float K_P = 70, std::function<bool(void)> func = {});

    void goTurn(float angleDegrees, std::function<bool(void)> func = {});
    void goTurnU(float universalAngleDegrees, std::function<bool(void)> func = {});
    bool goTurnVision(Goal goal, bool defaultClockwise, directionType cameraDir, float maxTurnAngle);
    float goTurnVision2(Goal goal, directionType cameraDir, float minSpeed, float timeout);
    void alignToGoalVision(Goal goal, bool clockwise, directionType cameraDirection, int timeout, float maxSpeed = 40);
    
    void goTurnFast(bool isClockwise, float turnDegrees, float maxSpeed, float minSpeed, float slowDownDegrees, float endSlowDegrees = 0,
    float timeout = 5, std::function<bool(void)> func = {});

    void goTurnFastU(float universalAngleDegrees, float maxSpeed, float minSpeed, float slowDownDegrees, float endSlowDegrees = 0, 
    float timeout = 5, std::function<bool(void)> func = {});

    // -------- OLD FUNCTIONS -----------
    void smartDrive(float distInches, float speed, directionType left, directionType right, int timeout, float slowDownInches, 
                    float turnPercent, bool stopAfter, std::function<bool(void)> func);
    void driveTurn(float degrees, float speed, bool isClockwise, int timeout, float slowDownInches = 10, 
                    bool stopAfter = true, std::function<bool(void)> func = {});
    void driveCurved(float distInches, float speed, directionType dir, int timeout, 
                      float slowDownInches, float turnPercent, bool stopAfter = true, std::function<bool(void)> func = {});
    void driveStraight(float distInches, float speed, directionType dir, int timeout, 
                      float slowDownInches, bool stopAfter = true, std::function<bool(void)> func = {});
    void driveStraightTimed(float speed, directionType dir, int timeMs, bool stopAfter = true, std::function<bool(void)> func = {});

    void updateCamera(Goal goal);

    

  private:


    void driveTeleop();
    void armTeleop();
    void pneumaticsTeleop();
    void clawMovement();
    

    float fourStart, chainStart;

    bool armHold = false;

    bool isSkills;
    bool armCoast = false;
};
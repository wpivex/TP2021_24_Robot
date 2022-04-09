#pragma once
#include "TrapezoidalController.cpp"
#include "PIDController.cpp"
#include "Buttons.cpp"
#include <string>
#include <fstream>
#include <vector>
#include <utility> // std::pair
#include <stdexcept> // std::runtime_error
#include <sstream> // std::stringstream
#include <string>
#include <functional>
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

    motor rightArm1;
    motor rightArm2;
    motor leftArm1;
    motor leftArm2;  

    // pot chainBarPot = pot(Brain.ThreeWirePort.H);
    // bumper fourBarBump = bumper(Brain.ThreeWirePort.A);

    digital_out frontGoal = digital_out(Brain.ThreeWirePort.B);
    digital_out backGoal = digital_out(Brain.ThreeWirePort.A);

    digital_out clawPiston = digital_out(Brain.ThreeWirePort.C);

    controller* robotController;

    inertial gyroSensor;
    gps gpsSensor;

    Buttons buttons;

    bool calibrationDone = false;

    enum DriveType { ONE_STICK_ARCADE, TWO_STICK_ARCADE, TANK };
    DriveType driveType;

    enum ControllerMapping {DEFAULT_MAPPING};
    ControllerMapping cMapping;
    Buttons::Button FRONT_CLAMP_TOGGLE, BACK_CLAMP_TOGGLE, CLAW_TOGGLE; 

    void setControllerMapping(ControllerMapping mapping);

    void callibrateGyro();

    void openClaw();
    void closeClaw();
    void goalClamp();
    void setFrontClamp(bool intaking);
    void setBackClamp(bool intaking);
    void stopArm();
    void setArmPercent(directionType d, double percent);

    void userControl( void );
    void teleop( void );
    void setLeftVelocity(directionType d, double percent);
    void setRightVelocity(directionType d, double percent);
    void stopLeft();
    void stopRight();
    void setBrakeType(brakeType b);
    void setMaxArmTorque(float c);
    void setMaxDriveTorque(float c);

    vision getCamera(directionType dir, Goal goal);


    // Drive Functions
    void goForward(float distInches, float maxSpeed, float rampUpInches = 0, float slowDownInches = 5,
      int timeout = 5, std::function<bool(void)> func = {}, bool stopAfter = true);
    void goForwardUniversal(float distInches, float maxSpeed, float universalAngle, float rampUpInches = 0, 
      float slowDownInches = 5, int timeout = 5, std::function<bool(void)> func = {});

    // Turning Functions
    void goTurnU(float universalAngleDegrees, std::function<bool(void)> func = {});
    void goTurn(float angleDegrees, std::function<bool(void)> func = {});
    void encoderTurn(float angle);
    void encoderTurnU(float universalAngleDegrees);
    void cursedTurn(float angle, float speed);

    // Curves
    void goRadiusCurve(float radius, float distAlongCircum, bool curveDirection, float maxSpeed, float rampUp, float slowDown, 
      bool stopAfter = true, float timeout = 5);
    void goCurve(float distInches, float maxSpeed, float turnPercent, float rampUpInches = 0, float slowDownInches = 5,
      int timeout = 5, std::function<bool(void)> func = {});
    void gyroCurve(float distInches, float maxSpeed, float turnAngle, int timeout, bool stopAfter = true, std::function<bool(void)> func = {});
    
    // Vision Functions
    void goVision(float distInches, float maxSpeed, Goal goal, directionType cameraDir, 
      float rampUpInches = 0, float slowDownInches = 5, int timeout = 5, bool stopAfter = true, float K_P = 70, std::function<bool(void)> func = {});
    bool goTurnVision(Goal goal, bool defaultClockwise, directionType cameraDir, float maxTurnAngle);
    void alignToGoalVision(Goal goal, bool clockwise, directionType cameraDirection, int timeout, float maxSpeed = 40);
    void updateCamera(Goal goal);
    
    // Misc.
    void driveArmDown(float timeout);
    void resetArmRotation();
    bool setArmDegrees(float degrees, float speed = 100);
    bool startArmFunc();

  private:

    float globalEncoderLeft;
    float globalEncoderRight;
    void driveTeleop();
    void armTeleop();
    void pneumaticsTeleop();
    void clawMovement();
    float getEncoderDistance();
    void resetEncoderDistance();

    bool driveHold = false;

    bool teleopArmLimited = false;

    bool isSkills;
};
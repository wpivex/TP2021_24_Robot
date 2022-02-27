#pragma once
#include "vex.h"
#include "Buttons.h"
#include <vector>
#include <list>
#include <string>
#include <cstring>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include "constants.h"
#include "PIDController.cpp"

using namespace vex;


const int NUM_NODES = 13;

class ArmGraph {

  public:

    enum Arm { START = 0, PLACE_GOAL_WITH_YELLOW = 1, INTER_ABOVE_ALLIANCE = 2, ABOVE_GOAL = 3, BACK_RING = 4, PLATFORM_HEIGHT = 5, INTAKE = 6,
        INTAKE_TO_PLACE_INTER_4 = 7, INTAKE_TO_PLACE_INTER_3 = 8, INTAKE_TO_PLACE_INTER_2 = 9, INTAKE_TO_PLACE_INTER_1 = 10, INTAKE_TO_PLACE_INTER_5 = 11,
        PLACE_GOAL_NO_YELLOW = 12
    };

    ArmGraph();
    void init(Buttons* bh, motor chainL, motor chainR, motor fourL, motor fourR, pot chainBarP, bumper fourBarB);
    void initArmPosition();
    void setArmDestination(Arm armPos);
    void moveArmToPosition(Arm armPos, float baseSpeed = 100, float timeout = 100);
    bool armMovement(bool buttonInput = true, float baseSpeed = 100);
    bool armMovementAuton();
    bool isMoving();

    void setFourVelocity(directionType d, double percent);
    void setChainVelocity(directionType d, double percent);
    void finishArmMovement();

  private:
    motor fourBarLeft;
    motor fourBarRight;
    motor chainBarLeft;
    motor chainBarRight;
    pot *chainBarPot;
    bumper *fourBarBump;

    float fourStart, chainStart;
    
    double angles[NUM_NODES][2] = {{0, 18.005371}, // START
      {19.200000, 212.158203}, // PLACE_GOAL_WITH_YELLOW
      {76.800000, 221.435547}, // INTER_ABOVE_ALLIANCE
      {76.800000, 221.435547}, // ABOVE_GOAL
      {900.000000, 194.580078}, // BACK_RING
      {508.000000, 110.473633}, // PLATFORM_HEIGHT
      {188.400000, 110.595703}, // INTAKE
      {690.000000, 162.475586}, // INTAKE_TO_PLACE_INTER_4 
      {796.800000, 124.206543}, // INTAKE_TO_PLACE_INTER_3 
      {752.800000, 100.952148}, // INTAKE_TO_PLACE_INTER_2 
      {680.000000, 101.074219}, // INTAKE_TO_PLACE_INTER_1 
      {239.600000, 209.960938}, // INTAKE_TO_PLACE_INTER_5
      {19.200000, 212.158203} // PLACE_GOAL_NO_YELLOW
    };

    int teleopMap [NUM_NODES];

    bool nodeEnabled[NUM_NODES];
      
    std::vector<int> adj[NUM_NODES];
    std::vector<int> togglableEdge[NUM_NODES];
    Buttons* buttons;

    std::vector<int> armPath;
    int targetArmPathIndex = 0;
    int targetNode = 2;
    std::string pathStr;

    bool arrived = true;
    bool arrivedFinal = true;
    float previousBaseSpeed;
    PID chainPID;

    bool chainBarDone;

    float fourBarVelocity, chainBarVelocity;
    directionType fourDir, chainDir;

    int startTimeout = vex::timer::system();
    int startNode;

    void calculateVelocities(float baseSpeed);

    void addEdge(int u, int v, bool togglable);
    void generateShortestPath(int start, int dest);

    std::string getPathStr();
    bool BFS(std::vector<int> adj[], int src, int dest, int pred[]);

};
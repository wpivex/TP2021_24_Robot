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
    void setPotInit(float pos);

  private:
    motor fourBarLeft;
    motor fourBarRight;
    motor chainBarLeft;
    motor chainBarRight;
    pot *chainBarPot;
    bumper *fourBarBump;

    float fourStart, chainStart;
    
    double angles[NUM_NODES][2] = {{10.800000, -3.600000}, // START
      {10.800000, -3.600000}, // PLACE_GOAL_WITH_YELLOW
      {42.000000, -39.200000}, // INTER_ABOVE_ALLIANCE
      {42.000000, -39.200000}, // ABOVE_GOAL
      {428.800000, 86.600000}, // BACK_RING
      {271.400000, 498.600000}, // PLATFORM_HEIGHT
      {70.400000, 559.200000}, // INTAKE
      {386.000000, 239.200000}, // INTAKE_TO_PLACE_INTER_4 
      {431.200000, 432.600000}, // INTAKE_TO_PLACE_INTER_3 
      {370.400000, 574.000000}, // INTAKE_TO_PLACE_INTER_2 
      {271.400000, 498.600000}, // INTAKE_TO_PLACE_INTER_1 
      {177.600000, 86.000000}, // INTAKE_TO_PLACE_INTER_5
      {10.800000, -3.600000} // PLACE_GOAL_NO_YELLOW
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
    float potInit;

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
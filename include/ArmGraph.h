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
    
    double angles[NUM_NODES][2] = {{-8.400000, 207.458496}, // START
      {-8.400000, 207.458496}, // PLACE_GOAL_WITH_YELLOW
      {51.600000, 214.904785}, // INTER_ABOVE_ALLIANCE
      {51.600000, 214.904785}, // ABOVE_GOAL
      {458.800000, 169.189453}, // BACK_RING
      {264.600000, 98.815918}, // PLATFORM_HEIGHT
      {79.000000, 102.661133}, // INTAKE
      {401.000000, 118.225098}, // INTAKE_TO_PLACE_INTER_4 
      {394.600000, 109.436035}, // INTAKE_TO_PLACE_INTER_3 
      {385.200000, 96.862793}, // INTAKE_TO_PLACE_INTER_2 
      {301.600000, 89.111328}, // INTAKE_TO_PLACE_INTER_1 
      {287.800000, 164.855957}, // INTAKE_TO_PLACE_INTER_5
      {-8.400000, 207.458496} // PLACE_GOAL_NO_YELLOW
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
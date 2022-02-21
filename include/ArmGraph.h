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
    void init(Buttons* bh, vex::motor chainL, vex::motor chainR, vex::motor fourL, vex::motor fourR);
    void initArmPosition();
    void setArmDestination(Arm armPos);
    void moveArmToPosition(Arm armPos, float baseSpeed = 100);
    bool armMovement(bool buttonInput = true, float baseSpeed = 100);
    bool armMovementAuton();
    bool isMoving();

    void setFourVelocity(directionType d, double percent);
    void setChainVelocity(directionType d, double percent);

  private:
    motor fourBarLeft;
    motor fourBarRight;
    motor chainBarLeft;
    motor chainBarRight;

    float fourStart, chainStart;
    
    double angles[NUM_NODES][2] = {{0, 0}, // START
      {389.600000, -69.600000}, // PLACE_GOAL_WITH_YELLOW
      {180.000000, -133.600000}, // INTER_ABOVE_ALLIANCE
      {552.800000, 43.600000}, // ABOVE_GOAL
      {1458.800000, 12.800000}, // BACK_RING
      {884.400000, 1040.400000}, // PLATFORM_HEIGHT
      {483.200000, 1224.800000}, // INTAKE
      {1068.400000, 480.000000}, // INTAKE_TO_PLACE_INTER_4
      {1357.600000, 1286.400000}, // INTAKE_TO_PLACE_INTER_3
      {1361.200000, 932.400000}, // INTAKE_TO_PLACE_INTER_2
      {1220.800000, 1298.800000}, // INTAKE_TO_PLACE_INTER_1
      {849.200000, 221.600000}, // INTAKE_TO_PLACE_INTER_5
      {5.600000, -196.000000} // PLACE_GOAL_NO_YELLOW
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
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

using namespace vex;


const int NUM_NODES = 13;

class ArmGraph {

  public:

  enum Arm { START = 0, PLACE_GOAL_NO_YELLOW = 1, INTER_ABOVE_ALLIANCE = 2, ABOVE_GOAL = 3, BACK_RING = 4, PLATFORM_PLACE = 5, INTAKE = 6,
      PLACE_GOAL_WITH_YELLOW = 7, INTAKE_TO_PLACE_INTER_1 = 8, INTAKE_TO_PLACE_INTER_2 = 9, INTAKE_TO_PLACE_INTER_3 = 10, INTAKE_TO_PLACE_INTER_4 = 11,
      INTAKE_TO_PLACE_INTER_5 = 12 };

    ArmGraph();
    void init(Buttons* bh, vex::motor chainL, vex::motor chainR, vex::motor fourL, vex::motor fourR);
    void initArmPosition();
    void setArmDestination(Arm armPos);
    void moveArmToPosition(Arm armPos);
    bool armMovement(bool buttonInput = true);
    bool armMovementAuton();

  private:
    motor fourBarLeft;
    motor fourBarRight;
    motor chainBarLeft;
    motor chainBarRight;

    float fourStart, chainStart;
    
    double angles[NUM_NODES][2] = {{0, 0}, // START
      {44.8, -180.8}, // PLACE_GOAL_NO_YELLOW
      {145.6, -121.6}, // INTER_ABOVE_ALLIANCE
      {539.2, -121.2}, // ABOVE_GOAL
      {1307.2, 39.2}, // BACK_RING
      {727.2, 1106.4}, // PLATFORM_PLACE
      {356.4, 920.4}, // INTAKE
      {380.8, 76.8}, // PLACE_GOAL_WITH_YELLOW
      {871.6, 1156.8}, // INTAKE_TO_PLACE_INTER_1
      {1065.6, 1243.2}, // INTAKE_TO_PLACE_INTER_2
      {1344.0, 1242.8}, // INTAKE_TO_PLACE_INTER_3
      {1436.8, 719.2}, // INTAKE_TO_PLACE_INTER_4
      {455.2, 152.4} // INTAKE_TO_PLACE_INTER_5
    };

    int teleopMap [NUM_NODES];

    bool nodeEnabled[NUM_NODES];
      
    std::vector<int> adj[NUM_NODES];
    Buttons* buttons;

    std::vector<int> armPath;
    int targetArmPathIndex = 0;
    int targetNode = 2;
    std::string pathStr;

    bool arrived = true;

    void addEdge(int u, int v);
    void generateShortestPath(int start, int dest);

    std::string getPathStr();
    bool BFS(std::vector<int> adj[], int src, int dest, int pred[]);

};
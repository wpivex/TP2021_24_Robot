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
      {0, 18.005371}, // PLACE_GOAL_WITH_YELLOW
      {338.400000, 15.991211}, // INTER_ABOVE_ALLIANCE
      {338.400000, 15.991211}, // ABOVE_GOAL
      {898.400000, 0.610352}, // BACK_RING
      {534.000000, 2.990723}, // PLATFORM_HEIGHT
      {143.200000, 249.938965}, // INTAKE
      {764.800000, 0.610352}, // INTAKE_TO_PLACE_INTER_4 
      {862.800000, 2.990723}, // INTAKE_TO_PLACE_INTER_3 
      {694.800000, 249.938965}, // INTAKE_TO_PLACE_INTER_2 
      {592.000000, 249.938965}, // INTAKE_TO_PLACE_INTER_1 
      {498.400000, 0.610352}, // INTAKE_TO_PLACE_INTER_5
      {0, 18.005371} // PLACE_GOAL_NO_YELLOW
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
#pragma once
#include "vex.h"
#include "Buttons.h"
#include <vector>
#include <list>

using namespace vex;
brain Brain;

const int NUM_NODES = 9;

class ArmGraph {

  public:
    ArmGraph();
    void init(Buttons* bh, vex::motor chainL, vex::motor chainR, vex::motor fourL, vex::motor fourR);
    void initArmPosition();
    void armMovement();
    void addEdge(int u, int v);
    void generateShortestPath(int start, int dest);

  private:
    motor fourBarLeft;
    motor fourBarRight;
    motor chainBarLeft;
    motor chainBarRight;

    float fourStart, chainStart;

    enum Arm { INTAKING = 0, INTER_INNER = 1, RING_FRONT = 2, ABOVE_MIDDLE = 3, RING_BACK = 4, PLACE_GOAL = 5, INTER_FRONT = 6, PLATFORM_LEVEL = 7, START = 9 };
    
    double angles[NUM_NODES][2] = {{394, 1140}, //intaking (0)
                          {1492.4, 682}, //intermediate 1 (1) (farther into robot)
                          {1060, 428}, //ring front (2)
                          {1375.2, 563.2}, //ring middle (3)
                          {1446, -26}, //ring back (4)
                          {509.2, 140}, //place goal (5)
                          {1339.2, 1777.6}, //intermediate 2 (6) (farther out of robot)
                          {870.4, 1180.2}, //score goal on platform (7)
                          {0, 0}}; //starting position

    int teleopMap [NUM_NODES];

    bool nodeEnabled[NUM_NODES] = {true, true, true, true, true, true, true, true};
      
    std::vector<int> adj[NUM_NODES];
    Buttons* buttons;

    std::vector<int> armPath;
    int targetArmPathIndex = 0;
    int targetNode = 2;

    bool arrived = true;

    bool BFS(std::vector<int> adj[], int src, int dest, int pred[]);

};
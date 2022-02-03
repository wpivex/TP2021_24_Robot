#include "ArmGraph.h"


ArmGraph::ArmGraph() : fourBarLeft(0), fourBarRight(0), chainBarLeft(0), chainBarRight(0) {}

void ArmGraph::init(Buttons* bh, vex::motor chainL, vex::motor chainR, vex::motor fourL, vex::motor fourR) {
  buttons = bh;
  fourBarLeft = fourL;
  fourBarRight = fourR;
  chainBarLeft = chainL;
  chainBarRight = chainR;
  bool isSkills = false;

  // Initialize teleop button mappings. Add / remove / change mappings as needed.
  std::fill_n(teleopMap, NUM_NODES, -1);
  std::fill_n(nodeEnabled, NUM_NODES, true);
  teleopMap[Buttons::DOWN] = INTAKE;
  // teleopMap[Buttons::Y] = RING_FRONT;
  teleopMap[Buttons::A] = ABOVE_GOAL;
  teleopMap[Buttons::X] = BACK_RING;
  teleopMap[Buttons::B] = isSkills? PLACE_GOAL_NO_YELLOW : PLACE_GOAL_WITH_YELLOW; //needs to toggle
  teleopMap[Buttons::RIGHT] = PLATFORM_PLACE;

  addEdge(START, ABOVE_GOAL);
  addEdge(START, INTER_ABOVE_ALLIANCE);
  addEdge(START, PLACE_GOAL_NO_YELLOW);
  addEdge(START, PLACE_GOAL_WITH_YELLOW);
  addEdge(START, ABOVE_GOAL);
  addEdge(START, BACK_RING);
  addEdge(INTER_ABOVE_ALLIANCE, PLACE_GOAL_NO_YELLOW);
  addEdge(INTER_ABOVE_ALLIANCE, PLACE_GOAL_WITH_YELLOW);
  addEdge(INTER_ABOVE_ALLIANCE, ABOVE_GOAL);
  addEdge(INTER_ABOVE_ALLIANCE, BACK_RING);
  addEdge(ABOVE_GOAL, PLATFORM_PLACE);
  addEdge(ABOVE_GOAL, BACK_RING);
  addEdge(PLATFORM_PLACE, INTAKE);
  
  addEdge(PLATFORM_PLACE, INTAKE_TO_PLACE_INTER_1);
  addEdge(INTAKE_TO_PLACE_INTER_1, INTAKE_TO_PLACE_INTER_2);
  addEdge(INTAKE_TO_PLACE_INTER_2, INTAKE_TO_PLACE_INTER_3);
  addEdge(INTAKE_TO_PLACE_INTER_3, INTAKE_TO_PLACE_INTER_4);
  addEdge(INTAKE_TO_PLACE_INTER_4, INTAKE_TO_PLACE_INTER_5);
  addEdge(INTAKE_TO_PLACE_INTER_5, PLACE_GOAL_WITH_YELLOW);

  // Initialize starting position and path
  targetNode = START;
}

void ArmGraph::initArmPosition() {
  // Reset position of motors
  chainBarLeft.resetPosition();
  fourBarLeft.resetPosition();
  chainBarRight.resetPosition();
  fourBarRight.resetPosition();

  // Store starting location of arm motors for purposes of velocity calculation
  fourStart = fourBarLeft.position(vex::degrees);
  chainStart = chainBarLeft.position(vex::degrees); 

}

// Set destination for arm, so that nonblocking armMovement can be called later in a loop
void ArmGraph::setArmDestination(Arm armPos) {
  generateShortestPath(targetNode, armPos);
}

void ArmGraph::moveArmToPosition(Arm armPos) {
  //Blocking function to move to a position
  generateShortestPath(targetNode, armPos);
  while(!armMovement(false)) {
    wait(20,msec);
  }
  
}

bool ArmGraph::armMovementAuton() {
  return armMovement(false);
}

// LOOK HOW FUCKING SHORT AND CLEAN THIS IS
// buttonInput is whether armMovement should be reading the controller button presses
bool ArmGraph::armMovement(bool buttonInput) {

  if (arrived) {

    // b is the button that was just pressed, or NONE if no button pressed this frame
    Buttons::Button b = buttons->get();

    // a relevant button was pressed, so set arm destination
    if (b != Buttons::NONE && teleopMap[b] != -1 && buttonInput) {
      generateShortestPath(targetNode, teleopMap[b]);
    }

    // not at final destination, so since it's arrived at the current one, set the new target destination to the next on the route
    if (armPath.size() > 1 && targetArmPathIndex != armPath.size() - 1) {
        targetArmPathIndex++;
        targetNode = armPath.at(targetArmPathIndex);
    }
    fourStart = fourBarLeft.position(vex::degrees);
    chainStart = chainBarLeft.position(vex::degrees);

  }

  
  log("%d %d  |  %s", targetNode, arrived ? 1 : 0, pathStr.c_str());

  float MARGIN = 10; // margin of error for if robot arm is in vicinity of target node
  float BASE_SPEED = 100; // Base speed of arm

  // Execute motor rotation towards target!
  int chainBarVelocity = BASE_SPEED * fabs((chainStart - angles[targetNode][1])/(fourStart - angles[targetNode][0]));
  fourBarLeft.rotateTo(angles[targetNode][0], vex::degrees, BASE_SPEED, vex::velocityUnits::pct, false);
  fourBarRight.rotateTo(angles[targetNode][0], vex::degrees, BASE_SPEED, vex::velocityUnits::pct, false);
  chainBarLeft.rotateTo(angles[targetNode][1], vex::degrees, chainBarVelocity , vex::velocityUnits::pct, false);
  chainBarRight.rotateTo(angles[targetNode][1], vex::degrees, chainBarVelocity , vex::velocityUnits::pct, false);

  // Calculate whether motor has arrived to intended target within some margin of error
  int delta1 = fabs(fourBarLeft.rotation(vex::degrees) - angles[targetNode][0]);
  int delta2 = fabs(chainBarLeft.rotation(vex::degrees) - angles[targetNode][1]);
  arrived = delta1 < MARGIN && delta2 < MARGIN;
  
  return arrived && targetArmPathIndex == armPath.size() - 1;
}

void ArmGraph::addEdge(int u, int v) {

  adj[u].push_back(v);
  adj[v].push_back(u);

}

// a modified version of BFS that stores predecessor
// of each vertex in array p
bool ArmGraph::BFS(std::vector<int> adj[], int src, int dest, int pred[]) {
    // a queue to maintain queue of vertices whose
    // adjacency list is to be scanned as per normal
    std::list<int> queue;
 
    // boolean array visited[] which stores the
    // information whether ith vertex is reached
    // at least once in the Breadth first search
    bool visited[NUM_NODES];
 
    // initially all vertices are unvisited
    // so v[i] for all i is false
    // and as no path is yet constructed
    for (int i = 0; i < NUM_NODES; i++) {
        visited[i] = false;
        pred[i] = -1;
    }
 
    // now source is first to be visited
    visited[src] = true;
    queue.push_back(src);
 
    // standard BFS algorithm
    while (!queue.empty()) {
        int u = queue.front();
        queue.pop_front();
        for (int i = 0; i < adj[u].size(); i++) {

            int n = adj[u][i]; // current neighbor

            // Skip disabled nodes
            if (!nodeEnabled[n]) continue;

            if (!visited[n]) {
                visited[n] = true;
                pred[n] = u;
                queue.push_back(n);
 
                // We stop BFS when we find
                // destination.
                if (n == dest)
                    return true;
            }
        }
    }
 
    return false;
}



std::string ArmGraph::getPathStr() {
  std::string str;
  bool start = true;
  for (int node : armPath) {
    char a;
     itoa(node, &a, 10);
     if (!start) {
       str += "->";
     }
     start = false;
    str += std::string(&a);
  }
  return str;
}
 
// Given start and end node, update the arm path to be the shortest path between the two nodes
void ArmGraph::generateShortestPath(int start, int dest) {

  targetArmPathIndex = 0;
  arrived = true;

  // predecessor[i] array stores predecessor of
  // i and distance array stores distance of i
  // from s
  int pred[NUM_NODES];

  if (!BFS(adj, start, dest, pred)) {
      return;
  }

  // vector path stores the shortest path
  int crawl = dest;
  armPath.clear();
  armPath.push_back(crawl);
  while (pred[crawl] != -1) {
      armPath.push_back(pred[crawl]);
      crawl = pred[crawl];
  }

  std::reverse(armPath.begin(),armPath.end());
  pathStr = getPathStr();


}
#include "ArmGraph.h"


ArmGraph::ArmGraph()
  : fourBarLeft(0), fourBarRight(0), chainBarLeft(0), chainBarRight(0) {
  
}

void ArmGraph::init(Buttons* bh, vex::motor chainL, vex::motor chainR, vex::motor fourL, vex::motor fourR) {
  buttons = bh;
  fourBarLeft = fourL;
  fourBarRight = fourR;
  chainBarLeft = chainL;
  chainBarRight = chainR;

  // Initialize teleop button mappings. Add / remove / change mappings as needed.
  std::fill_n(teleopMap, NUM_NODES, -1);
  teleopMap[Buttons::DOWN] = INTAKING;
  teleopMap[Buttons::Y] = RING_FRONT;
  teleopMap[Buttons::A] = ABOVE_MIDDLE;
  teleopMap[Buttons::X] = RING_BACK;
  teleopMap[Buttons::B] = PLACE_GOAL;
  teleopMap[Buttons::RIGHT] = PLATFORM_LEVEL;
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

  addEdge(INTAKING, PLACE_GOAL);
  addEdge(PLACE_GOAL, INTER_FRONT);
  addEdge(INTER_FRONT, INTER_INNER);
  addEdge(INTER_INNER, ABOVE_MIDDLE);
  addEdge(ABOVE_MIDDLE, PLACE_GOAL);
  addEdge(ABOVE_MIDDLE, RING_BACK);
}

// LOOK HOW FUCKING SHORT AND CLEAN THIS IS
void ArmGraph::armMovement() {

  if (arrived) {

    // b is the button that was just pressed, or NONE if no button pressed this frame
    Buttons::Button b = buttons->get();

    // a relevant button was pressed, so set arm destination
    if (b != Buttons::NONE && teleopMap[b] != -1) {
      generateShortestPath(targetNode, teleopMap[b]);
      targetArmPathIndex = 1;
    }

    // not at final destination, so since it's arrived at the current one, set the new target destination to the next on the route
    if (targetArmPathIndex != armPath.size() - 1) {
        targetArmPathIndex++;
    }
    fourStart = fourBarLeft.position(vex::degrees);
    chainStart = chainBarLeft.position(vex::degrees);
  }

  targetNode = armPath.at(targetArmPathIndex);

  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("%d %d %d %d %d", targetNode, arrived ? 1 : 0);

  float MARGIN = 10; // margin of error for if robot arm is in vicinity of target node
  float BASE_SPEED = 30; // Base speed of arm

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
 
// utility function to print the shortest distance
// between source vertex and destination vertex
void ArmGraph::generateShortestPath(int start, int dest) {

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

}
#include "ArmGraph.h"


ArmGraph::ArmGraph() : fourBarLeft(0), fourBarRight(0), chainBarLeft(0), chainBarRight(0) {}

void ArmGraph::init(Buttons* bh, vex::motor chainL, vex::motor chainR, vex::motor fourL, vex::motor fourR) {
  buttons = bh;
  fourBarLeft = fourL;
  fourBarRight = fourR;
  chainBarLeft = chainL;
  chainBarRight = chainR;

  // Initialize teleop button mappings. Add / remove / change mappings as needed.
  std::fill_n(teleopMap, NUM_NODES, -1);
  std::fill_n(nodeEnabled, NUM_NODES, true);
  teleopMap[Buttons::DOWN] = INTAKE;
  // teleopMap[Buttons::Y] = RING_FRONT;
  teleopMap[Buttons::A] = ABOVE_GOAL;
  teleopMap[Buttons::X] = BACK_RING;
  teleopMap[Buttons::B] = PLACE_GOAL_WITH_YELLOW; //isSkills? PLACE_GOAL_NO_YELLOW : PLACE_GOAL_WITH_YELLOW; //need to make toggleable
  teleopMap[Buttons::RIGHT] = PLATFORM_HEIGHT;
  teleopMap[Buttons::LEFT] = INTAKE_TO_PLACE_INTER_2;

  addEdge(START, PLACE_GOAL_WITH_YELLOW, false);
  addEdge(PLACE_GOAL_WITH_YELLOW, INTER_ABOVE_ALLIANCE, false);
  addEdge(INTER_ABOVE_ALLIANCE, ABOVE_GOAL, false);
  addEdge(BACK_RING, ABOVE_GOAL, false);
  addEdge(INTAKE, PLATFORM_HEIGHT, false);
  addEdge(PLACE_GOAL_WITH_YELLOW, ABOVE_GOAL, false);
  addEdge(START, BACK_RING, false);
  addEdge(PLACE_GOAL_WITH_YELLOW, BACK_RING, false);
  addEdge(INTER_ABOVE_ALLIANCE, BACK_RING, false);
  addEdge(ABOVE_GOAL, START, false);
  addEdge(INTER_ABOVE_ALLIANCE, START, false);
  addEdge(INTAKE_TO_PLACE_INTER_1, INTAKE_TO_PLACE_INTER_2, false);
  addEdge(INTAKE_TO_PLACE_INTER_2, INTAKE_TO_PLACE_INTER_3, false);
  addEdge(INTAKE_TO_PLACE_INTER_3, INTAKE_TO_PLACE_INTER_4, false);
  addEdge(ABOVE_GOAL, INTAKE_TO_PLACE_INTER_5, false);
  addEdge(INTAKE_TO_PLACE_INTER_5, INTAKE_TO_PLACE_INTER_4, false);
  addEdge(INTAKE_TO_PLACE_INTER_1, PLATFORM_HEIGHT, false);
  addEdge(ABOVE_GOAL, BACK_RING, false);

  addEdge(ABOVE_GOAL, PLATFORM_HEIGHT, true);
  addEdge(INTER_ABOVE_ALLIANCE, PLATFORM_HEIGHT, true);
  addEdge(PLACE_GOAL_WITH_YELLOW, PLATFORM_HEIGHT, true);
  addEdge(INTAKE_TO_PLACE_INTER_1, INTER_ABOVE_ALLIANCE, true);
  addEdge(INTAKE_TO_PLACE_INTER_1, ABOVE_GOAL, true);
  addEdge(PLACE_GOAL_WITH_YELLOW, INTAKE_TO_PLACE_INTER_2, true);
  addEdge(INTER_ABOVE_ALLIANCE, INTAKE_TO_PLACE_INTER_2, true);
  addEdge(ABOVE_GOAL, INTAKE_TO_PLACE_INTER_2, true);
  addEdge(PLACE_GOAL_WITH_YELLOW, INTAKE_TO_PLACE_INTER_3, true);
  addEdge(INTER_ABOVE_ALLIANCE, INTAKE_TO_PLACE_INTER_3, true);
  addEdge(ABOVE_GOAL, INTAKE_TO_PLACE_INTER_3, true);
  addEdge(PLATFORM_HEIGHT, INTAKE_TO_PLACE_INTER_3, true);
  addEdge(ABOVE_GOAL, INTAKE_TO_PLACE_INTER_4, true);
  addEdge(INTER_ABOVE_ALLIANCE, INTAKE_TO_PLACE_INTER_4, true);
  addEdge(PLACE_GOAL_WITH_YELLOW, INTAKE_TO_PLACE_INTER_4, true);
  addEdge(INTAKE_TO_PLACE_INTER_1, PLACE_GOAL_WITH_YELLOW, true);

  chainBarLeft.setBrake(hold);
  chainBarRight.setBrake(hold);
  fourBarLeft.setBrake(hold);
  fourBarRight.setBrake(hold);


  // Initialize starting position and path
  targetNode = START;
}

void ArmGraph::initArmPosition() {
  // Reset position of motors
  chainBarLeft.resetPosition();
  fourBarLeft.resetPosition();
  chainBarRight.resetPosition();
  fourBarRight.resetPosition();

}

bool ArmGraph::isMoving() {
  return !arrivedFinal;
}

// Set destination for arm, so that nonblocking armMovement can be called later in a loop
void ArmGraph::setArmDestination(Arm armPos) {
  if (targetNode == armPos) return;
  generateShortestPath(targetNode, armPos);
}

void ArmGraph::moveArmToPosition(Arm armPos, float baseSpeed, int timeout) {

  if (arrivedFinal && targetNode == armPos) return;

  int startTime = vex::timer::system();

  //Blocking function to move to a position
  generateShortestPath(targetNode, armPos);
  while(!isTimeout(startTime, timeout) && !armMovement(false, baseSpeed)) {
    wait(20,msec);
  }
  if (isTimeout(startTime, timeout)) {
    chainBarLeft.stop();
    chainBarRight.stop();
    fourBarLeft.stop();
    fourBarRight.stop();
  }
  
}

bool ArmGraph::armMovementAuton() {
  log("abc");
  return armMovement(false);
}

void ArmGraph::calculateVelocities(float baseSpeed) {

  fourStart = fourBarLeft.position(vex::degrees);
  chainStart = chainBarLeft.position(vex::degrees);

  float dChain = fabs(chainStart - angles[targetNode][1]);
  float dFour = fabs(fourStart - angles[targetNode][0]);

  if (dFour > dChain) {
    fourBarVelocity = baseSpeed;
    chainBarVelocity = baseSpeed * dChain / dFour;
  } else {
    chainBarVelocity = baseSpeed;
    fourBarVelocity = baseSpeed * dFour / dChain;
  }

  fourDir = (fourBarLeft.rotation(vex::degrees) < angles[targetNode][0]) ? forward : reverse;
  chainDir = (chainBarLeft.rotation(vex::degrees) < angles[targetNode][1]) ? forward : reverse;

}

// LOOK HOW SHORT AND CLEAN THIS IS
// Future Ansel: I lied this is pain
// buttonInput is whether armMovement should be reading the controller button presses
bool ArmGraph::armMovement(bool buttonInput, float baseSpeed) {

  float MARGIN = 21.0; // margin of error for if robot arm is in vicinity of target node

  // Timeout, revert to the last position, if teleop
  if (buttonInput && vex::timer::system() - startTimeout > ARM_TIMEOUT) {
      generateShortestPath(targetNode, startNode);
  }

  if (arrived) {

    startTimeout = vex::timer::system();

    // b is the button that was just pressed, or NONE if no button pressed this frame
    Buttons::Button b = buttons->get();

    // a relevant button was pressed, so set arm destination
    if (b != Buttons::NONE && teleopMap[b] != -1 && buttonInput && targetNode != teleopMap[b]) {
      generateShortestPath(targetNode, teleopMap[b]);
    }

    // not at final destination, so since it's arrived at the current one, set the new target destination to the next on the route
    if (armPath.size() > 1 && targetArmPathIndex != armPath.size() - 1) {
        targetArmPathIndex++;
        targetNode = armPath.at(targetArmPathIndex);
        chainBarDone = false;
        calculateVelocities(baseSpeed);
    }
  }


  if (arrivedFinal) {
    chainBarLeft.stop();
    chainBarRight.stop();
    fourBarLeft.stop();
    fourBarRight.stop();
    return true;
  }


  // Calculate whether motor has arrived to intended target within some margin of error
  //int delta1 = fabs(fourBarLeft.rotation(vex::degrees) - angles[targetNode][0]);
  //int delta2 = fabs(chainBarLeft.rotation(vex::degrees) - angles[targetNode][1]);
  //log("%d %d %d %d", delta1, delta2, fourBarDone ? 1 : 0, chainBarDone ? 1 : 0);

  if (targetArmPathIndex == armPath.size() - 1) {
    fourBarLeft.rotateTo(angles[targetNode][0], vex::degrees, fourBarVelocity, vex::velocityUnits::pct, false);
    fourBarRight.rotateTo(angles[targetNode][0], vex::degrees, fourBarVelocity, vex::velocityUnits::pct, false);
  } else {
    fourBarLeft.spin(fourDir, fourBarVelocity, vex::velocityUnits::pct);
    fourBarRight.spin(fourDir, fourBarVelocity, vex::velocityUnits::pct);
  }

  chainBarLeft.rotateTo(angles[targetNode][1], vex::degrees, chainBarVelocity, vex::velocityUnits::pct, false);
  chainBarRight.rotateTo(angles[targetNode][1], vex::degrees, chainBarVelocity, vex::velocityUnits::pct, false);


  arrived = fabs(fourBarLeft.rotation(vex::degrees) - angles[targetNode][0]) < MARGIN;
  
  arrivedFinal = arrived && (targetArmPathIndex == armPath.size() - 1);

  log("%d %d %d |  %s", targetNode, arrived ? 1 : 0, arrivedFinal ? 1 : 0, pathStr.c_str());

  return arrivedFinal;

}

void ArmGraph::addEdge(int u, int v, bool togglable) {

  adj[u].push_back(v);
  togglableEdge[u].push_back(togglable);

  adj[v].push_back(u);
  togglableEdge[v].push_back(togglable);

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

            // Skip disabled edges and nodes
            if (!IS_SKILLS && (togglableEdge[u][i] || !nodeEnabled[n])) continue;

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

  startNode = start;

  targetArmPathIndex = 0;
  arrived = true;
  arrivedFinal = false;

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

// Set in volts to circumvent internal PID (which detracts from our custom PID)
void ArmGraph::setFourVelocity(directionType d, double percent) {
  fourBarLeft.spin(d, percent / 100.0 * MAX_VOLTS, voltageUnits::volt);
  fourBarRight.spin(d, percent / 100.0 * MAX_VOLTS, voltageUnits::volt);
}

void ArmGraph::setChainVelocity(directionType d, double percent) {
  chainBarLeft.spin(d, percent / 100.0 * MAX_VOLTS, voltageUnits::volt);
  chainBarRight.spin(d, percent / 100.0 * MAX_VOLTS, voltageUnits::volt);
}
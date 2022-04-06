#if !defined(MYLIB_CONSTANTS_H)
#define MYLIB_CONSTANTS_H 1

#include "vex.h"

using namespace vex;

const bool IS_SKILLS = false;
const bool testingArm = false;

brain Brain;
controller Controller1(controllerType::primary);

struct Goal {
  int id;
  int bright;
  vision::signature sig;
};

// COMP FIELD
// const struct Goal YELLOW = {0, 21, vex::vision::signature (1, 2023, 3393, 2708, -3631, -3185, -3408, 2.500, 0)};
// const struct Goal RED = {1, 41, vex::vision::signature (1, 8435, 10495, 9464, -1029, -641, -836, 3.000, 0)};
// const struct Goal BLUE = {2, 52, vex::vision::signature (1, -2657, -1837, -2247, 7385, 11983, 9684, 3.000, 0)};

namespace ARM_CURRENT {
  const static float OFF = 0.0;
  const static float LOW = 0.1;
  const static float MID = 1.0;
  const static float HIGH = 10.0;
}

// // SKILLS
// const struct Goal YELLOW = {0, 25, vex::vision::signature (1, 2401, 3421, 2911, -3187, -2789, -2988, 2.500, 0)};
// const struct Goal RED = {1, 50, vex::vision::signature (1, 7787, 9783, 8785, -773, -457, -615, 3.000, 0)};
// const struct Goal BLUE = {2, 102, vex::vision::signature (1, -2617, -1735, -2176, 6659, 12361, 9510, 3.000, 0)};

const struct Goal YELLOW = {0, 28, vex::vision::signature (1, 2069, 3277, 2673, -3269, -2915, -3092, 2.500, 0)};
// const struct Goal RED = {1, 56, vex::vision::signature (1, 5767, 9395, 7581, -685, 1, -342, 3.000, 0)};
const struct Goal RED = {1, 49, vex::vision::signature (1, 7951, 10657, 9304, -1067, -549, -808, 3.000, 0)}; // I added this feb 22
const struct Goal BLUE = {2, 80, vex::vision::signature (1, -3063, -1681, -2372, 6893, 12701, 9797, 3.000, 0)};

const float MAX_VOLTS = 12.0; // maximum volts for vex motors


static const float VISION_CENTER_X = 157.0;
static const float DIST_BETWEEN_WHEELS = 15.0;

static const float FORWARD_MIN_SPEED = 15; // the robot approaches this speed at the end of going forward
static const float TURN_MIN_SPEED = 5; // the robot approaches this speed at the end of turning

static const int ARM_TIMEOUT = 300000;


static inline float distanceToDegrees(float distInches) {
  return distInches * 360 / 2 / M_PI / (4 / 2); // 4 in diameter wheels
}

static inline float degreesToDistance(float degrees) {
  return degrees / (360 / 2 / M_PI / (4 / 2)); // 4 in diameter wheels
}

// return distance in inches if wanting to turn turnAngle degrees
static inline float getTurnAngle(float turnAngle) {

  return fabs(distanceToDegrees(turnAngle / 360 * 2 * M_PI * (15.125 / 2)));
}

// timeout in seconds
static inline bool isTimeout(int startTime, float timeout) {
  return timeout != -1 && vex::timer::system() >= startTime + (int) (timeout*1000.0);
}



// log output to controller
template <class ... Args>
static inline void logController(char *f, Args ... args) {

  char *format = (char*)f;

  Controller1.Screen.clearScreen();
  int row = 1;

  char* pch = strtok (f,"\n");
  while (pch != NULL)
  {
    Controller1.Screen.setCursor(row, 1);
    Controller1.Screen.print(pch, args...);
    pch = strtok (NULL, "\n");
    row++;
  }

}

// log output to brain display the way you would with printf
// log output to brain display the way you would with printf
// DO NOT LOG WITH TRAILING NEWLINE OR YOU GET MEM PERM ERR
template <class ... Args>
static inline void log(int line, const char *f, Args ... args) {
  Brain.Screen.clearLine(line);
  int row = line;

  char buffer[200];
  sprintf(buffer, f, args...);

  char* pch = strtok (buffer,"\n");
  while (pch != NULL)
  {
    Brain.Screen.setCursor(row, 1);
    Brain.Screen.print(pch);
    pch = strtok (NULL, "\n");
    row++;
  }
}

template <class ... Args>
static inline void log(const char *f, Args ... args) {
  log(1, f, args ...);
}

#endif
#if !defined(MYLIB_CONSTANTS_H)
#define MYLIB_CONSTANTS_H 1

#include "vex.h"

struct Goal {
  int bright;
  vex::vision::signature sig;
};

namespace {}

const struct Goal YELLOW = {13, vex::vision::signature (1, 1849, 2799, 2324, -3795, -3261, -3528, 2.500, 0)};
const struct Goal RED = {56, vex::vision::signature (1, 5767, 9395, 7581, -685, 1, -342, 3.000, 0)};
const struct Goal BLUE = {67, vex::vision::signature (1, -2675, -1975, -2324, 8191, 14043, 11116, 3.000, 0)};


static const float VISION_CENTER_X = 157.0;
static const float DIST_BETEWEEN_WHEELS = 15.0;

static const float FORWARD_MIN_SPEED = 20; // the robot approaches this speed at the end of going forward
static const float TURN_MIN_SPEED = 20; // the robot approaches this speed at the end of turning


static inline float distanceToDegrees(float distInches) {
  return distInches * 360 / 2 / M_PI / (4 / 2) * 15 / 14; // 4 in diameter wheels
}


// return distance in inches if wanting to turn turnAngle degrees
static inline float getTurnAngle(float turnAngle) {

  return fabs(distanceToDegrees(turnAngle / 360 * 2 * M_PI * (15.125 / 2)));
}

// timeout in seconds
static inline bool isTimeout(int startTime, int timeout) {
  return timeout != -1 && vex::timer::system() >= startTime + timeout*1000;
}

#endif
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

static int distanceToDegrees(float dist) {
  return dist * 360 / 2 / M_PI / (4 / 2) * 15 / 14; // 4 in diameter wheels
}

// return distance in inches if wanting to turn turnAngle degrees
static int getTurnAngle(float turnAngle) {

  return fabs(distanceToDegrees(turnAngle / 360 * 2 * M_PI * (15.125 / 2)));

}

#endif
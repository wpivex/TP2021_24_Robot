// INCLUDE RIGHT VERSION OF ROBOT (15 OR 24)
// #include "main15.cpp"
#include "main24.cpp"

using namespace vex;

int main() {

  Competition.bStopAllTasksBetweenModes = true;
  return mainFunc();

}

# TP2021_24_Robot

**November 6:**
- Working vision detection for yellow object
- Follow yellow object from reverse orientation
- Sensor color callibration

**November 7:**
- Working robot blind turn into vision detection forwards and collecting goal
- Refactor teleop arm code into teleop()
- Generic arm function that can be utitilized by both teleop and auton
- Blocking function to move arm to position for auton

**November 9:**
- Callibrate arm position states for 24" robot
- Finish and test teleop arm code
- Make CSV obsolete and hardcode arm lookup table

**Todo for teleop arm:**
- Refine teleop arm with speed, position, and fine tuning arm state locations
- Test auton arm
**
Todo for vision:**
- We probably should tilt the vision sensor downwards anyways. It works, but it will be more reliable and less prone to distractions if it is
- We should calculate dx, or change in centerX. This value should not exceed some threshold (meaning taht the robot has now locked onto a different object) and instead will revert to visionless drive straight
- We should still try to test/fix my bounded vision detection. If it works, it will completely eliminate any possibility of veering to the wrong thing

**General goals:**
- Optimize initial push to center goal for speed
- Write auton code for robot to turn and pick up next goal
- Write auton code for 15" robot

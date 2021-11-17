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

**November 11:**
- Tweak initial blind curve towards goal
- Tweak PID turning
- Implement vision turning
- Add turn to left goal, collect goal, then turn to base for auton
- Identify problematic robot design in exceeding 36" horizontal limit

**November 16**:
- Refactor blind turn and vision turn for blocking and nonblocking options
- Test auton arm
- Make progress on concurrency between robot turn and arm raise

#include "vex.h"
#include "math.h"

using namespace vex;
using namespace std;

const double AcceptableErrorCoefficient = 1.0/10; //Sensor distance values and distance to wall values get multiplied by this value before being rounded to create acceptable error. Range: (0,infinity).

void CorrectRotation(int Angle) {
  //the argument angle is RotationCorrectionInDegrees and needs to be used to get all the sensor readings to be approximately equal to one another (within AcceptableAngleError).
}

void MoveToCenter(int Dist) {
  //the argument dist is DistToHallCenter and needs to be used with PID (proportional integral derivative) controller to shift the robot. This will be hard. I am thinking that we need to calculate DistToHallCenter as this maneuver is being done, but this will rely on our distances to "show up" accurately (not be out of range due to turning). 
}

void ASSInit() {
  //I am treating RangeFinderA as top left, R.F.B as top right, R.F.C as bottom right, R.F.D as bottom left (think of it as clockwise ABCD starting at top left).
  //Also all these sonars are currently configured to ports A,B,C,D respectively which may need to be changed (in src/robot-config.cpp and include/robot-config.h).
  while(true) {
    int RotationCorrectionInDegrees; //someone needs to do the math, maybe I will. I am invisioning this to be a value == in magnitude to the angle that the robot is at in the hallway (assume walls are perfectly parallel, all 4 sensors output distances, and angle 0 is when the robot is perfectly parallel to walls)
    CorrectRotation(RotationCorrectionInDegrees);
    
    int DistToHallCenter = round((FrontRightSonar.distance(mm) + BackRightSonar.distance(mm) - FrontLeftSonar.distance(mm) - BackLeftSonar.distance(mm)) / 4 * AcceptableErrorCoefficient); //Right direction is defined as positive. P.S. you can do the math this is what the expression comes out to be.
    MoveToCenter(DistToHallCenter);
  }
}
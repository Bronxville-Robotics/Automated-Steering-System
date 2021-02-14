#include "vex.h"
#include "math.h"

using namespace vex;
using namespace std;

const int RobotWidthCM = 1; //someone needs to measure the robot width, use metric system.
const int RobotLengthCM = 1; //someone needs to measure the robot length, use metric system.

int IntegralOfDistToTarget = 0;
int PrevDistToTarget;

int DistToTarget(int LA, int LB, int LC, int LD) {
  //the L arguments are the distances wrt each sensor.
  int AngleBetweenSensorsAndWall = atan(2*RobotWidthCM / (abs(LA-LD)+abs(LB-LC)));
  int HallWidthCM = sin(AngleBetweenSensorsAndWall) * ((LA+LB+LC+LD)/2+RobotWidthCM);
  return 0; //someone needs to calculate the distance from the center of the robot to the center of the hall and return it. You will need hall width, the angle above, one of the L args, and  one robot dimension measurement.
}

void AdjustMotorSpeedsWithPID(int Dist) {
  //LeftMotor.setVelocity();   
  //RightMotor.setVelocity();
  //use PID (proportional integral derivative) controller to alter the speed of each driving motor wrt the other motor (don't alter the combined motor speeds). update IntegralOfDistToTarget and use that as integral componenet of PID.
  //update PrevDistToTarget and use it with the current Dist to approximate the derivative.
}

void ASSInit() {
  //I am treating RangeFinderA as top left, R.F.B as top right, R.F.C as bottom right, R.F.D as bottom left, R.F.E as front facing (think of it as clockwise ABCD starting at top left with finder E between A and B).
  //Also all these sonars are currently configured to ports A,B,C,D,E respectively which may need to be changed (in src/robot-config.cpp and include/robot-config.h).
  while(not RangeFinderE.foundObject()) {
    double LA = RangeFinderA.distance(mm);
    double LB = RangeFinderB.distance(mm);
    double LC = RangeFinderC.distance(mm);
    double LD = RangeFinderD.distance(mm);
    int Dist = DistToTarget(LA,LB,LC,LD);

    AdjustMotorSpeedsWithPID(Dist);
    
    //The hope with the following is that once the velocities are altered and the spin functions called, the motors will spin at those velocities. If that is not the case, fix it.
    LeftMotor.spin(fwd);
    RightMotor.spin(fwd);
  }
}

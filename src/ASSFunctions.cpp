#include "vex.h"
#include "cmath"

using namespace vex;
using namespace std;

const int distanceBetweenSideSensorPairs = 1; //someone needs to measure the robot width in mm.

int integralOfDistanceToTarget = 0;
int previousDistanceToTarget;

double distanceToTarget(int frontLeft, int frontRight, int backLeft, int backRight) {
  double angleBetweenSensorsAndWallInDegrees = atan(2.0*distanceBetweenSideSensorPairs / (abs(frontLeft-backLeft + backRight-frontRight)));
  return sin(angleBetweenSensorsAndWallInDegrees)/4.0 * (frontRight-frontLeft + backRight-backLeft); 
}

void adjustMotorSpeedsWithPID(int distance) {
  //LeftMotor.setVelocity();   
  //RightMotor.setVelocity();
  //use PID (proportional integral derivative) controller to alter the speed of each driving motor wrt the other motor (don't alter the combined motor speeds). update IntegralOfDistToTarget and use that as integral componenet of PID.
  //we should consider using a library to implement PID: https://github.com/tekdemo/MiniPID
  //update PrevDistToTarget and use it with the current Dist to approximate the derivative.
}

void initASS() {
  //I added LeftMotor and RightMotor to the robot config files in place of the previous SmartTurnSomethings. I also added FrontFacingSonar and configured it to port B which may need to be changed.
  while(true) { 
    double lengthFrontLeftSonar = FrontLeftSonar.distance(mm);
    double lengthFrontRightSonar = FrontRightSonar.distance(mm);
    double lengthBackLeftSonar = BackLeftSonar.distance(mm);
    double lengthBackRightSonar = BackRightSonar.distance(mm);
    double distance = distanceToTarget(lengthFrontLeftSonar, lengthFrontRightSonar, lengthBackLeftSonar, lengthBackRightSonar);

    adjustMotorSpeedsWithPID(distance);
    
    //The hope with the following member functions is that every time the velocities are altered in AdjustMotorSpeedsWithPID, the motors will spin at those velocities (given the front sonar doesn't detect a wall). 
    if(not FrontFacingSonar.foundObject()) {
      LeftMotor.spin(fwd);
      RightMotor.spin(fwd);
    }
    else {
      LeftMotor.stop();
      RightMotor.stop();
    }
  }
}

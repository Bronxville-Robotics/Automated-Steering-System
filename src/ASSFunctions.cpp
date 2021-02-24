#include "vex.h"
#include "cmath"

using namespace vex;
using namespace std;

const int DistBetweenSideSensorPairs = 1; //someone needs to measure the robot width in mm.

int IntegralOfDistToTarget = 0;
int PrevDistToTarget;

double DistToTarget(int FL, int FR, int BL, int BR) {
  double AngleBetweenSensorsAndWallDeg = atan(2.0*DistBetweenSideSensorPairs / (abs(FL-BL + BR-FR)));
  return sin(AngleBetweenSensorsAndWallDeg)/4.0 * (FR-FL + BR-BL); 
}

void AdjustMotorSpeedsWithPID(int Dist) {
  //LeftMotor.setVelocity();   
  //RightMotor.setVelocity();
  //use PID (proportional integral derivative) controller to alter the speed of each driving motor wrt the other motor (don't alter the combined motor speeds). update IntegralOfDistToTarget and use that as integral componenet of PID.
  //we should consider using a library to implement PID: https://github.com/tekdemo/MiniPID
  //update PrevDistToTarget and use it with the current Dist to approximate the derivative.
}

void ASSInit() {
  //I added LeftMotor and RightMotor to the robot config files in place of the previous SmartTurnSomethings. I also added FrontFacingSonar and configured it to port B which may need to be changed.
  while(true) { 
    double LengthFLS = FrontLeftSonar.distance(mm);
    double LengthFRS = FrontRightSonar.distance(mm);
    double LengthBLS = BackLeftSonar.distance(mm);
    double LengthBRS = BackRightSonar.distance(mm);
    double Dist = DistToTarget(LengthFLS, LengthFRS, LengthBLS, LengthBRS);

    AdjustMotorSpeedsWithPID(Dist);
    
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

this line should raise an error
#include "vex.h"
#include "math.h"

using namespace vex;
using namespace std;

const int RobotWidthCM = 1; //someone needs to measure the robot width, use metric system.
const int RobotLengthCM = 1; //someone needs to measure the robot length, use metric system.

int IntegralOfDistToTarget = 0;
int PrevDistToTarget;

int DistToTarget(int FL, int FR, int BL, int BR) {
  //the arguments are the distances wrt each sensor.
  int AngleBetweenSensorsAndWall = atan(2*RobotWidthCM / (abs(FL-BL)+abs(FR-BR)));
  int HallWidthCM = sin(AngleBetweenSensorsAndWall) * ((FL+FR+BL+BR)/2+RobotWidthCM);
  return 0; //someone needs to calculate the displacement from the center of the robot to the center of the hall and return it (lets say right of target is positive). You will need hall width, the angle above, one of the args, and  one robot dimension measurement.
}

void AdjustMotorSpeedsWithPID(int Dist) {
  //LeftMotor.setVelocity();   
  //RightMotor.setVelocity();
  //use PID (proportional integral derivative) controller to alter the speed of each driving motor wrt the other motor (don't alter the combined motor speeds). update IntegralOfDistToTarget and use that as integral componenet of PID.
  //update PrevDistToTarget and use it with the current Dist to approximate the derivative.
}

void ASSInit() {
  //I added LeftMotor and RightMotor to the robot config files in place of the previous SmartTurnSomethings. I also added FrontFacingSonar and configured it to port B which may need to be changed.
  while(true) { 
    double LengthFLS = FrontLeftSonar.distance(mm);
    double LengthFRS = FrontRightSonar.distance(mm);
    double LengthBLS = BackLeftSonar.distance(mm);
    double LengthBRS = BackRightSonar.distance(mm);
    int Dist = DistToTarget(LengthFLS, LengthFRS, LengthBLS, LengthBRS);

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

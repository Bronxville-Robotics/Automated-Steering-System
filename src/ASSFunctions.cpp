#include "vex.h"
#include "cmath"
#include <vector>

using namespace vex;
using namespace std;

const int robotWidthCM = 1; //someone needs to measure the robot width, use metric system.

//Coefficients to weight the proportional, integral, and derivative components of PID
//Initialized at 1 but should be determined experimentally
const double P = 1;
const double I = 1;
const double D = 1;
const double speedFactor = 1; // One factor to control slowing down/speeding up the effect of the entire PID.
const double baseMotorSpeed = 30; //Original Motor Speed is set to 30 rpm.  Should be experimentally played with.

vector<double> errors; //List of all recorded error measurements to determine integral and derivative.

int integralOfDistanceToTarget = 0;
int previousDistanceToTarget;
double currentError;

//Determine the integral by summing all collected error measurments
//Approximation of the integral
double sumErrors() {
  double sum = 0;
  for(auto i = errors.begin(); i != errors.end(); i++) {
    sum += *i;
  }
  return sum;
}

//Determine the derivative by subtracting the past error measurement from the current (y2 - y1)
//Approximation of the derivative
double errorDerivative() {
  double previousError = errors[errors.size() - 2];
  double currentError = errors[errors.size() - 1];
  return currentError - previousError;
}

double distanceToTarget(double frontLeft, double frontRight, double backLeft, double backRight) {
  //the arguments are the distances wrt each sensor.
  int angleBetweenSensorsAndWall = atan(2*robotWidthCM / (abs(frontLeft-backLeft)+abs(frontRight-backRight)));
  int hallWidthCM = sin(angleBetweenSensorsAndWall) * ((frontLeft+frontRight+backLeft+backRight)/2+robotWidthCM);
  return 0; //someone needs to calculate the displacement from the center of the robot to the center of the hall and return it (lets say right of target is positive). You will need hall width, the angle above, one of the args, and  one robot dimension measurement.
}

//Determines the amount the motor speeds should change (in RPM) based on PID
//Assumes the right side of the hallway is positive and left is negative.
void adjustMotorSpeedsWithPID(int distance) {
  double proportion = distance;
  double integral = sumErrors();
  double derivative = errorDerivative();

  double change = (P*proportion + I*integral + D*derivative) * speedFactor;

  RightMotor.setVelocity(baseMotorSpeed + change, rpm);
  LeftMotor.setVelocity(baseMotorSpeed - change, rpm);
  //use PID (proportional integral derivative) controller to alter the speed of each driving motor wrt the other motor (don't alter the combined motor speeds). update IntegralOfDistToTarget and use that as integral componenet of PID.
  //we should consider using a library to implement PID: https://github.com/tekdemo/MiniPID
  //update PrevDistToTarget and use it with the current Dist to approximate the derivative.
}

void initASS() {
  //I added LeftMotor and RightMotor to the robot config files in place of the previous SmartTurnSomethings. I also added FrontFacingSonar and configured it to port B which may need to be changed.
  //Adds an initial reading to the errors list so that derivative and integral can be computed without error.
  errors.push_back(distanceToTarget(FrontLeftSonar.distance(mm), FrontRightSonar.distance(mm), BackLeftSonar.distance(mm), BackRightSonar.distance(mm)));

  while(true) { 
    double lengthFrontLeftSonar = FrontLeftSonar.distance(mm);
    double lengthFrontRightSonar = FrontRightSonar.distance(mm);
    double lengthBackLeftSonar = BackLeftSonar.distance(mm);
    double lengthBackRightSonar = BackRightSonar.distance(mm);

    currentError = distanceToTarget(lengthFrontLeftSonar, lengthFrontRightSonar, lengthBackLeftSonar, lengthBackRightSonar);
    errors.push_back(currentError);
    adjustMotorSpeedsWithPID(currentError);
    
    LeftMotor.spin(fwd);
    RightMotor.spin(fwd);
    
    wait(100, msec);
  }
}

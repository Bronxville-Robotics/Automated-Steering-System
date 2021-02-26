#include "vex.h"
#include "cmath"
#include <vector>

using namespace vex;
using namespace std;

const int distanceBetweenSideSensorPairs = 610;

//Coefficients to weight the proportional, integral, and derivative components of PID.
//Initialized at 1 but should be determined experimentally.
const double P = 0.01;
const double I = 0;
const double D = 0;
const double speedFactor = 1; // One factor to control slowing down/speeding up the effect of the entire PID.
const double baseMotorSpeed = 14; //Original Motor Speed is set to 30 rpm.  Should be experimentally played with.
const double maxRpm = 30; //Rpm value we want the robot to cap out at.

vector<double> errors; //List of all recorded error measurements to determine integral and derivative.

double currentError;

//Determine the integral by summing all collected error measurments.
//Approximation of the integral.
double sumErrors() {
  double sum = 0;
  for(auto i = errors.begin(); i != errors.end(); i++) {
    sum += *i;
  }
  return sum;
}

//Determine the derivative by subtracting the past error measurement from the current (y2 - y1).
//Approximation of the derivative.
double errorDerivative() {
  double previousError = errors[errors.size() - 2];
  double currentError = errors[errors.size() - 1];
  return currentError - previousError;
}

//If the center of the robot is left of target (ie. left of hall center) returns positive distance in mm. If right of center, returns negative distance in mm.
double distanceToTarget(int frontLeft, int frontRight, int backLeft, int backRight) {
  double angleBetweenSensorsAndWallInDegrees = atan(2.0*distanceBetweenSideSensorPairs / (abs(frontLeft-backLeft + backRight-frontRight)));
  return sin(angleBetweenSensorsAndWallInDegrees)/4.0 * (frontRight-frontLeft + backRight-backLeft); 
}

//Determines the amount the motor speeds should change (in RPM) based on PID.
void adjustMotorSpeedsWithPID(int distance) {
  double proportion = distance;
  double integral = sumErrors();
  double derivative = errorDerivative();

  double change = (P*proportion + I*integral + D*derivative) * speedFactor;

  if(change + baseMotorSpeed > maxRpm) {
    change = maxRpm - baseMotorSpeed; 
  }

  LeftMotor.setVelocity(baseMotorSpeed + change, rpm);
  RightMotor.setVelocity(baseMotorSpeed - change, rpm);
  //we should consider using a library to implement PID: https://github.com/tekdemo/MiniPID.
}

void initASS() {
  //Adds an initial reading to the errors list so that derivative and integral can be computed without error.
  errors.push_back(distanceToTarget(FrontLeftSonar.distance(mm), FrontRightSonar.distance(mm), BackLeftSonar.distance(mm), BackRightSonar.distance(mm)));

  while(true) { 
    double distanceFrontLeftSonar = FrontLeftSonar.distance(mm);
    double distanceFrontRightSonar = FrontRightSonar.distance(mm);
    double distanceBackLeftSonar = BackLeftSonar.distance(mm);
    double distanceBackRightSonar = BackRightSonar.distance(mm);

    Brain.Screen.printAt(1, 20, "Front Left Sonar: %f mm", distanceFrontLeftSonar);
    Brain.Screen.printAt(1, 40, "Front Right Sonar: %f mm", distanceFrontRightSonar);
    Brain.Screen.printAt(1, 60, "Back Left Sonar: %f mm", distanceBackLeftSonar);
    Brain.Screen.printAt(1, 80, "Back Right Sonar: %f mm", distanceBackRightSonar);

    currentError = distanceToTarget(distanceFrontLeftSonar, distanceFrontRightSonar, distanceBackLeftSonar, distanceBackRightSonar);
    Brain.Screen.printAt(1, 100, "Current Error: %f mm", currentError);

    errors.push_back(currentError);
    adjustMotorSpeedsWithPID(currentError);
    
    LeftMotor.spin(fwd);
    RightMotor.spin(fwd);
    
    wait(100, msec);
  }
}
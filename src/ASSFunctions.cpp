#include "vex.h"
#include "cmath"
#include <vector>

using namespace vex;
using namespace std;

const int distanceBetweenSideSensorPairs = 610;
const int driveTimeBeforeTurn = 2000;
const int driveTimeAfterTurn = 3000;

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

double distanceToTarget(int frontLeft, int frontRight, int backLeft, int backRight) {
  double angleBetweenSensorsAndWallInDegrees = atan(2.0*distanceBetweenSideSensorPairs / (abs(frontLeft-backLeft + backRight-frontRight)));
  return sin(angleBetweenSensorsAndWallInDegrees)/4.0 * (frontRight-frontLeft + backRight-backLeft); 
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

void hallToHallTurn(bool turnRight) {
  Drivetrain.drive(vex::forward, 60, rpm);
  task::sleep(driveTimeBeforeTurn);
  Drivetrain.stop();

  if (turnRight) {
    Drivetrain.turnFor(90, degrees, true);
  }
  else {
    Drivetrain.turnFor(-90, degrees, true);
  }

  Drivetrain.drive(vex::forward, 60, rpm);
  task::sleep(driveTimeAfterTurn);
  Drivetrain.stop();
}

void triggerASS() {
  //Adds an initial reading to the errors list so that derivative and integral can be computed without error.
  if (errors.empty()) {
    errors.push_back(distanceToTarget(FrontLeftSonar.distance(mm), FrontRightSonar.distance(mm), BackLeftSonar.distance(mm), BackRightSonar.distance(mm)));
  }

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
}

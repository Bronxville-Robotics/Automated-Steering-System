#include <vector>
#include "vex.h"

//Begins Automated Steering System
void ASSInit();

//Returns a vector representing a 360 degree scan
std::vector<bool> scan();

//Picks an angle of next path based on scan data parameter
//Param degreesOfScanSeparation refers to the gap between individual scans
double findTurnAngle(std::vector<bool> data, double degreesOfScanSeparation);

//Turns the robot to the specified angle
void turnToAngle(double angle);

//Drives the robot forward until an object is detected in front
void driveUntilWall();


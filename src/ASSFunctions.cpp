#include "vex.h"
#include <vector>

using namespace vex;

static int refreshRate = 60;
static int numDataPts = 12;

std::vector<bool> scan() {
  std::vector<bool> data;

  Rotator.resetPosition();

  for(int i = 0; i < numDataPts; i++) {
    
    data.push_back(Range.foundObject());
    Rotator.rotateTo(360 / numDataPts * i, degrees, true);
  }

  Rotator.rotateTo(0, degrees, false);

  return data;
}

double findTurnAngle(std::vector<bool> data) {
  double degreesOfScanSeparation = 360.0 / numDataPts;

  int run = 0;
  int longestRun = 0;
  int begin = 0;

  for(int i = 0; i < data.size(); i++) {

    if(!data.at(i)) {

      run++;
    }

    else {

      if(run > longestRun) {

        longestRun = run;
        begin = i - run;
      }

      run = 0;
    }
  }

  if(run > longestRun) {

    longestRun = run;
    begin = data.size() - run;
  }

  return (longestRun * degreesOfScanSeparation/2) + (begin * degreesOfScanSeparation);
}

void driveUntilWall() {
  bool objectFound = Range.foundObject();
  Drivetrain.drive(forward, 60, rpm);

  while (!objectFound) {
    objectFound = Range.foundObject();
    task::sleep(1000 / refreshRate);
  }

  Drivetrain.stop();
}

void turnToAngle(double angle) {

  Drivetrain.turnFor(angle, degrees, true);
}

void ASSInit() {
  driveUntilWall();

  std::vector<bool> data = scan();
  for (std::vector<bool>::const_iterator i = data.begin(); i != data.end(); i++) {
    Brain.Screen.print(*i);
  }

  double turnAngle = findTurnAngle(data);
  turnToAngle(turnAngle);
}


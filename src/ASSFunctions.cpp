#include "vex.h"
#include <vector>
#include <algorithm>
#include <cmath>

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
  int sz = data.size();
  vector<double> v(sz);
  vector<double> vNew(sz);

  for(int i = 0; i < sz; i++) {
    if(data[i]) {
      v[i] = 1;
    }
    else {
      v[i] = -1;
    }
  }

  for(int i = 0; i < sz; i++) {
    for(int j = 0; j < sz; j++) {
      vNew[i] += 1 / (std::abs(sz / 2.0 + i + j) + sz / 2.0) * v[j];
    }
    vNew[i] += sz / 2.0 - std::abs(i - sz / 2.0);
  }

  return double(std::min_element(vNew.begin(), vNew.end()) - vNew.begin()) / sz * 360;
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


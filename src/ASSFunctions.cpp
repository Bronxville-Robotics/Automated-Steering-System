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

double findTurnAngle(std::vector<bool> data) { //needs to be vector<double> v and erase first line after
  vector<double> v=data.replace(data.begin(), data.end(), 0, -1);
  vector<double> vNew(v.size());

  for(int i = 0; i < v.size(); i++) {
    for(int j = 0; j < v.size(); j++) {
      vNew[i] += 1 / (std::abs(v.size() / 2.0 + i + j) + v.size() / 2.0) * v[j];
    }
    vNew[i] += v.size()/2.0 - std::abs(i - v.size() / 2.0);
  }

  return find(vNew.begin(), vNew.end(), std::min_element(vNew.begin(), vNew.end())) / vNew.size() * 360; //needs to be double * 360, then round
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


#include "vex.h"
#include <vector>
#include <cmath>
#include <algorithm>

using namespace vex;
using namespace std;

static int refreshRate = 60;
static int numDataPts = 12;
static int detectionThresholdInCM = 60;

//lowers the detection range of Range.foundObject() such that the robot is less sensitive to being near walls
bool foundObjectWithThreshold() {
  return Range.foundObject() && Range.distance(distanceUnits::cm) < detectionThresholdInCM;
}

std::vector<bool> scan() {
  std::vector<bool> data;
  Rotator.resetPosition();
  double degreeIncrement = 360 / numDataPts;

  for(int i = 0; i < numDataPts; i++) {
    data.push_back(foundObjectWithThreshold());
    Rotator.rotateTo(degreeIncrement * i, rotationUnits::deg, true);
  }

  Rotator.rotateTo(0, rotationUnits::deg, true);

  return data;
}

double findTurnAngle(std::vector<bool> data) { 
  const double k = 1.0 / 6;
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
      vNew[i] += pow((std::abs(i - j) - sz / 2.0) / (sz / 2.0), 2) * v[j];
    }
    vNew[i] = vNew[i] / sz;
    vNew[i] -= std::abs(i - sz / 2.0) / (sz / 2.0) * k;
  }

  return double(std::min_element(vNew.begin(), vNew.end()) - vNew.begin()) / sz * 360;
}

void driveUntilWall() {
  bool objectFound = foundObjectWithThreshold();
  Drivetrain.drive(vex::forward, 60, rpm);

  while (!objectFound) {
    objectFound = foundObjectWithThreshold();
    task::sleep(1000 / refreshRate);
  }

  Drivetrain.stop();
}

void turnToAngle(double angle) {
  if(angle <= 180) {
    Drivetrain.turnFor(angle * -1, degrees, true);
  }

  else {

    Drivetrain.turnFor(360 - angle, degrees, true);
  }
}

int ASSInit() {
  
  while(true) {
  
    driveUntilWall();

    Drivetrain.stop();

    std::vector<bool> data = scan();

    for (std::vector<bool>::const_iterator i = data.begin(); i != data.end(); i++) {
      Brain.Screen.print(*i);
    }

    Brain.Screen.newLine();

    double turnAngle = findTurnAngle(data);
    turnToAngle(turnAngle);
  }
  
  return 0;
}
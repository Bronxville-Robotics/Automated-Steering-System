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
      vNew[i] += pow((std::abs(i - j) - sz / 2.0) / (sz / 2.0), 2) * v[j]; //passing absolute difference of iterators through quadratic function f(x)=(x-sz/2)^2 / (sz/2)^2 (where x is abs diff iterators, / (sz/2)^2 is to keep quadratic between 0 and 1 (over range [0,sz) )).
    }
    vNew[i] = vNew[i] / sz; //keeps the range between approximately -1/3 and approximately 1/3 for sz>10.
    vNew[i] += pow((i - sz / 2.0) / (sz / 2.0), 2) * k; //applying the same equation as before but this time to prioritize the frontmost values (increase the backmost element of vNew by 1*k, increase the frontmost element by 0*k). 
  }

  return double(std::min_element(vNew.begin(), vNew.end()) - vNew.begin()) / sz * 360; //return angle of minimum value in Vnew.
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


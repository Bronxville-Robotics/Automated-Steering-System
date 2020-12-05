#include "vex.h"
#include <vector>
#include <algorithm>
#include <cmath>

using namespace vex;
using namespace std;

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

/*
ABOUT findTurnAngle:

STEPS:
Takes in boolean vector data, sets sz to size of vector.
Creates two double vectors v and vNew of size sz.
Populates sz with 1s where data had Trues, -1s where data had Falses.

For every element in v, finds the absolute displacement between that element and every other element. 
There are sz number of these displacements per element.

For each element, passes the sz number of displacements, one at a time, into the function f(x)=(x-sz/2)^2 / (sz/2)^2 as x.
Then all of these outputs of f(x) are multiplied by the value of their respective elements in v and added to the respective element of vNew. 

For each element of vNew, passes the element value into the function g(x)=abs(x-sz/2) / (sz/2) * k as x. (k is used to amplify this function.)
Then all of these outputs of g(x) are subtracted from their respective element of vNew. 

The function returns the angle of the smallest element in Vnew, which is found by dividing the element index by sz and then multiplying by 360.

PURPOSE:
The function finds the best angle by finding the minimum of a new vector of values. 
These values are created by considering every scan and it's distance to every angle that the function can output. 
Scan values that are closer to the angle have more influence, while scan values farther away from the angle have almost no influence.
If there are a lot of scan values that are True and are close to an angle, that angle will likely not be selected since it's corresponding value will be high.

Besides considering values at other angles, the function also makes angles closer to 180 undesirable so that the robot is less likely to go back and forth.
It does this by decreasing the values closer to 0 / 360.
You can control how powerful this feature is by changing k. 
A value of 1.0 / 6 suits k well enough when the number of data points exceeds 10. 
*/

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


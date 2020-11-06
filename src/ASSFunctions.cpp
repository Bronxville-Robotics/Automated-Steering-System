#include "vex.h"
#include <vector>

using namespace vex;

static int refreshRate = 60;

std::vector<bool> scan() {
  std::vector<bool> data;

  Rotator.resetPosition();
  int numDataPts = 12;

  for(int i = 0; i < numDataPts; i++) {
    
    data.push_back(Range.foundObject());
    Rotator.rotateTo(360 / numDataPts * i, degrees, true);
  }

  Rotator.rotateTo(360, degrees, false);

  return data;
}

void turnToPath(std::vector<bool> data) {

}

void driveUntilWall() {
  bool objectFound = Range.foundObject();
  Drivetrain.drive(forward, 60, rpm);

  while (!objectFound) {
    objectFound = Range.foundObject();
    task::sleep(1000 / refreshRate);
  }

  Drivetrain.stop();

  std::vector<bool> data = scan();
  for (std::vector<bool>::const_iterator i = data.begin(); i != data.end(); i++) {
    Brain.Screen.print(*i);
    //Brain.Screen.print(" ");
  }
  
  //turnToPath(data);
}


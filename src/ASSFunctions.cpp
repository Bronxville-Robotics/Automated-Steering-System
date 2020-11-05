#include "vex.h"
#include <vector>

using namespace vex;

static int refreshRate = 60;

std::vector<bool> scan() {
  std::vector<bool> data;

  return data;
}

void turnToPath(std::vector<bool>) {

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
  turnToPath(data);
}


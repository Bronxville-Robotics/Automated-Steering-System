#include "vex.h"

using namespace vex;

static int refreshRate = 60;
static int detectionThresholdInCM = 60;
static int scannerAngle = 90;
static int turnAngle = 90;
static int driveTimeAfterNoObjectFound = 5000; 

//lowers the detection range of Range.foundObject() such that the robot is less sensitive to being near walls
bool foundObjectWithThreshold() {
  return Range.foundObject() && Range.distance(distanceUnits::cm) < detectionThresholdInCM;
}

void driveUntilClear() {
  bool objectFound = foundObjectWithThreshold();
  Drivetrain.drive(vex::forward, 60, rpm);

  while (objectFound) {
    objectFound = foundObjectWithThreshold();
    task::sleep(1000 / refreshRate);
  }

  Drivetrain.stop(); 
  return;
}

void driveFor(int time) {
  Drivetrain.drive(vex::forward, 60, rpm);
  task::sleep(time / refreshRate);
  Drivetrain.stop(); 
  return;
}

void turnToAngle(double angle) {
  if(angle <= 180) {
    Drivetrain.turnFor(angle * -1, degrees, true);
  }
  else {
    Drivetrain.turnFor(360 - angle, degrees, true);
  }
}

//drives forward while sees wall. After not seeing wall drives some more, then checks if wall is still not there. Based on that it either turns and drives some more or it restarts this loop.
void ASSInit() {
  Rotator.rotateTo(scannerAngle, rotationUnits::deg, true);

  while(true) {
    driveUntilClear();
    driveFor(driveTimeAfterNoObjectFound);

    if not foundObjectWithTreshold() {
      turnToAngle(turnAngle);
      driveFor(driveTimeAfterNoObjectFound);
    }
  }
}

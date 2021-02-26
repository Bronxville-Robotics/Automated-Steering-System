#include "vex.h"

using namespace vex;

const double tamper = 0.6;

double leftVelocity = 0;
double rightVelocity = 0;

void manualDrive() {

  leftVelocity = Controller1.Axis3.position() * tamper;
  rightVelocity = Controller1.Axis2.position() * tamper;

  LeftMotor.setVelocity(leftVelocity, pct);
  RightMotor.setVelocity(rightVelocity, pct);

  LeftMotor.spin(fwd);
  RightMotor.spin(fwd);
}
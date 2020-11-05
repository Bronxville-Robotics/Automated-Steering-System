/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\AJ                                               */
/*    Created:      Fri Oct 30 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Drivetrain           drivetrain    1, 2            
// RotatingRangeMotor   motor         3               
// Range                sonar         A, B            
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "ASSFunctions.h"
#include <iostream>

using namespace vex;

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
    
  //Drivetrain.drive(forward, 60, rpm);

  driveUntilWall();


}

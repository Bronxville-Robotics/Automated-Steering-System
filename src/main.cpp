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
// Drivetrain           drivetrain    12, 11          
// Rotator              motor         10              
// Range                sonar         A, B            
// ---- END VEXCODE CONFIGURED DEVICES ----

/*
Can we classify different parts of the hallway based on scan characteristics?
Can we do different levels of scan based on distance away?
*/

#include "vex.h"
#include "ASSFunctions.h"
#include <iostream>

using namespace vex;

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  ASSInit();
}

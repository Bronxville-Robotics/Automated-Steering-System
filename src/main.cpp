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
// Drivetrain           drivetrain    2, 9            
// Rotator              motor         10              
// Range                sonar         A, B            
// ---- END VEXCODE CONFIGURED DEVICES ----

/*
Can we classify different parts of the hallway based on scan characteristics?
Can we do different levels of scan based on distance away?
*/

#include "vex.h"
#include "ASSFunctions.h"

using namespace vex;

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
 // loop forever to always update the information displayed on the screen
while(1) {
    Brain.Screen.clearScreen();

    // display the sonar distance in mm on the screen
    Brain.Screen.printAt(1, 20, "distance: %f mm", Range.distance(distanceUnits::mm));
    
    // display the sonar distance in mm on the screen
    Brain.Screen.printAt(1, 40, "distance: %f in", Range.distance(distanceUnits::in));
    
    // display the sonar distance in mm on the screen
    Brain.Screen.printAt(1, 60, "distance: %f cm", Range.distance(distanceUnits::cm));
    
    //Sleep the task for a short amount of time to prevent wasted resources.
    task::sleep(500);
}
}

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
// FrontLeftSonar       sonar         A, B            
// FrontRightSonar      sonar         C, D            
// BackLeftSonar        sonar         E, F            
// BackRightSonar       sonar         G, H            
// LeftMotor            motor         2               
// RightMotor           motor         9               
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

/*
Can we classify different parts of the hallway based on scan characteristics?
Can we do different levels of scan based on distance away?
*/

#include "vex.h"
#include "ASSFunctions.h"
#include "manual-drive.h"

using namespace vex;

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  while(true) {

    manualDrive();

    wait(100,msec);    
  }
}
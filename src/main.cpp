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
// Expander10           triport       10              
// FrontSonar           sonar         G, H            
// ---- END VEXCODE CONFIGURED DEVICES ----

/*
Can we classify different parts of the hallway based on scan characteristics?
Can we do different levels of scan based on distance away?
*/

#include "vex.h"
#include "ASSFunctions.h"
#include "manual-drive.h"

using namespace vex;

bool manualOverrideIsEnabled = true;

void bPressed() {
  manualOverrideIsEnabled = !manualOverrideIsEnabled;
  Brain.Screen.clearScreen();
  LeftMotor.stop();
  RightMotor.stop();
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Controller1.ButtonB.pressed(bPressed);
  
  while(true) {
    while(!manualOverrideIsEnabled) {
      triggerASS();
      wait(100, msec);
    }

    while(manualOverrideIsEnabled){
      Brain.Screen.printAt(1, 20, "Manual Steering Enabled.");
      manualDrive();
    }
  }
}
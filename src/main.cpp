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

#include "vex.h"
#include "ASSFunctions.h"
#include "manual-drive.h"

using namespace vex;

bool manualOverrideIsEnabled = true;

void bPressed() {
  manualOverrideIsEnabled = !manualOverrideIsEnabled;

  if(manualOverrideIsEnabled) {
    task::stop(triggerASS);
    Brain.Screen.clearScreen();
    LeftMotor.stop();
    RightMotor.stop();
    task manualDriving(manualDrive);
    Brain.Screen.printAt(1, 20, "Manual Steering Enabled.");
  }

  else {
    task::stop(manualDrive);
    Brain.Screen.clearScreen();
    LeftMotor.stop();
    RightMotor.stop();
    task automaticDriving(triggerASS);
  }
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Controller1.ButtonB.pressed(bPressed);
  task manualDriving(manualDrive);
}
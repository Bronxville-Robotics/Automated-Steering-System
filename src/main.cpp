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
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "ASSFunctions.h"

using namespace vex;

std::string mode = "automatic";

void stopAllOperations() {

  Drivetrain.stop();
  Rotator.rotateTo(0, rotationUnits::deg, true);
}

int setModeStatusFunction() {

  while(true) {
  
    double startTime = Brain.timer(msec);

    while(Controller1.ButtonB.pressing()) {

      if(Brain.timer(msec) - startTime >= 2000) {

        if(mode.compare("automatic") == 0) {

          mode = "manual";
          Brain.Screen.newLine();
          Brain.Screen.print(mode.c_str());
          stopAllOperations();
          break;
        }

        else if(mode.compare("manual") == 0) {

          mode = "automatic";
          Brain.Screen.newLine();
          Brain.Screen.print(mode.c_str());
          stopAllOperations();
          break;
        }
      }

      wait(20, msec);
    }

    wait(20, msec);
  }

  return 0;
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  task setMode(setModeStatusFunction);

  while(true) {

    if(mode == "automatic") {

      task automaticDrive(ASSInit);

      while(mode == "automatic") {

        wait(20, msec);
      }
    }

    else if(mode == "manual") {

      task manualDrive(rc_auto_loop_function_Controller1);

      while(mode == "manual") {

        wait(20, msec);
      }
    }
  }
}
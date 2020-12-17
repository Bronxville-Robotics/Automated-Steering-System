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

int setModeStatusFunction() {

  while(true) {
  
    double startTime = Brain.timer(msec);

    while(Controller1.ButtonB.pressing()) {

      if(Brain.timer(msec) - startTime >= 2000) {

        if(mode.compare("automatic") == 0) {

          mode = "manual";
          Brain.Screen.newLine();
          Brain.Screen.print(mode.c_str());
          break;
        }

        else if(mode.compare("manual") == 0) {

          mode = "automatic";
          Brain.Screen.newLine();
          Brain.Screen.print(mode.c_str());
          break;
        }

        wait(20, msec);
        return 1;
      }
    }
    wait(5, msec);
  }
  return 0;
}


int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  //Automatic Drive
  task automaticDrive(ASSInit);

  //Mode Task Test
  task setMode(setModeStatusFunction);

  //Manual Drive Task Test
  task manualDrive(rc_auto_loop_function_Controller1);
  manualDrive.suspend();

  /*for(int i = 0; i < 10; i++) {

    ASSInit();
  }*/

  //Test #1 --> Does this work?
  /*while(true) {

    if(mode.compare("automatic") == 0) {

      manualDrive.suspend();
      Drivetrain.stop();
      automaticDrive.resume();
    }

    else if(mode.compare("manual") == 0) {

      automaticDrive.suspend();
      Drivetrain.stop();
      manualDrive.resume();
    }
    wait(50, msec);
  }*/

  //Test #2
  //Backup While Loop
  while(true) {

    if(mode.compare("automatic") == 0) {

      manualDrive.sleep(21);
      Drivetrain.stop();
    }

    else if(mode.compare("manual") == 0) {

      automaticDrive.sleep(21);
      Drivetrain.stop();
    }

    wait(20, msec);
  }

  //Test #3
  //New Steering
}

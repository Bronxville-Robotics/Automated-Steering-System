#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor LeftMotor = motor(PORT2, ratio18_1, true);
motor RightMotor = motor(PORT9, ratio18_1, false);
drivetrain Drivetrain = drivetrain(LeftMotor, RightMotor, 319.19, 295, 40, mm, 1);
sonar FrontLeftSonar = sonar(Brain.ThreeWirePort.A);
sonar FrontRightSonar = sonar(Brain.ThreeWirePort.C);
sonar BackLeftSonar = sonar(Brain.ThreeWirePort.E);
sonar BackRightSonar = sonar(Brain.ThreeWirePort.G);
sonar FrontFacingSonar = sonar(Brain.ThreeWirePort.B);

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}

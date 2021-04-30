#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
triport Expander10 = triport(PORT10);
sonar FrontLeftSonar = sonar(Brain.ThreeWirePort.A);
sonar FrontRightSonar = sonar(Brain.ThreeWirePort.C);
sonar BackLeftSonar = sonar(Brain.ThreeWirePort.E);
sonar BackRightSonar = sonar(Brain.ThreeWirePort.G);
motor LeftMotor = motor(PORT2, ratio18_1, false);
motor RightMotor = motor(PORT9, ratio18_1, true);
controller Controller1 = controller(primary);
sonar FrontSonar = sonar(Expander10.G);
drivetrain Drivetrain = drivetrain(LeftMotor, RightMotor);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}
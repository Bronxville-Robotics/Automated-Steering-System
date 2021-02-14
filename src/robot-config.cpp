#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor LeftMotor = motor(PORT2, ratio18_1, false);
motor RightMotor = motor(PORT9, ratio18_1, true);
drivetrain Drivetrain = drivetrain(LeftMotor, RightMotor, 319.19, 295, 40, mm, 1);
sonar RangeFinderA = sonar(Brain.ThreeWirePort.A);
sonar RangeFinderB = sonar(Brain.ThreeWirePort.B);
sonar RangeFinderC = sonar(Brain.ThreeWirePort.C);
sonar RangeFinderD = sonar(Brain.ThreeWirePort.D);
sonar RangeFinderE = sonar(Brain.ThreeWirePort.E);

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}
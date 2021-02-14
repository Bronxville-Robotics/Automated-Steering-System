using namespace vex;

extern brain Brain;

// VEXcode devices
extern drivetrain Drivetrain;
extern motor LeftMotor;
extern motor RightMotor;
extern sonar RangeFinderA;
extern sonar RangeFinderB;
extern sonar RangeFinderC;
extern sonar RangeFinderD;
extern sonar RangeFinderE;


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );
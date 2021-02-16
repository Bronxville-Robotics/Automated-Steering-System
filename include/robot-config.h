using namespace vex;

extern brain Brain;

// VEXcode devices
extern drivetrain Drivetrain;
extern motor LeftMotor;
extern motor RightMotor;
extern sonar FrontLeftSonar;
extern sonar FrontRightSonar;
extern sonar BackLeftSonar;
extern sonar BackRightSonar;
extern sonar FrontFacingSonar;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );

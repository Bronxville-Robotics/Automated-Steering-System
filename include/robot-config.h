using namespace vex;

extern brain Brain;

// VEXcode devices
extern sonar FrontLeftSonar;
extern sonar FrontRightSonar;
extern sonar BackLeftSonar;
extern sonar BackRightSonar;
extern motor LeftMotor;
extern motor RightMotor;
extern controller Controller1;
extern triport Expander10;
extern sonar FrontSonar;
extern drivetrain Drivetrain;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );
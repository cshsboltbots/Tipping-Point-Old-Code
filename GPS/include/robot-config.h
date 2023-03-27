using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern rotation Right_Sensor;
extern rotation Left_Sensor;
extern inertial Inertial;
extern motor_group Left;
extern motor_group Right;
extern gps GPS;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );
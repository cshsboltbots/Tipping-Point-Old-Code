using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor_group Right;
extern motor_group Left;
extern rotation Left_Sensor;
extern rotation Right_Sensor;
extern motor Ring;
extern motor Goal;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );
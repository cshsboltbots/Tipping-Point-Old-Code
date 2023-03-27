using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor_group Left;
extern motor_group Right;
extern rotation Right_Sensor;
extern rotation Left_Sensor;
extern motor Ring;
extern digital_out Grabber;
extern motor_group Arm;
extern inertial Inertial;
extern motor Red;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );
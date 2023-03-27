using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern controller Controller2;
extern motor_group Left;
extern motor_group Right;
extern motor Ring;
extern motor_group Lift;
extern rotation Left_Sensor;
extern rotation Right_Sensor;
extern inertial Inertial;
extern digital_out Single;
extern digital_out Double;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );
#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor LeftMotorA = motor(PORT1, ratio6_1, true);
motor LeftMotorB = motor(PORT4, ratio6_1, false);
motor_group Left = motor_group(LeftMotorA, LeftMotorB);
motor RightMotorA = motor(PORT2, ratio6_1, false);
motor RightMotorB = motor(PORT3, ratio6_1, true);
motor_group Right = motor_group(RightMotorA, RightMotorB);
rotation Right_Sensor = rotation(PORT11, false);
rotation Left_Sensor = rotation(PORT12, true);
inertial Inertial = inertial(PORT13);
gps GPS = gps(PORT10, 0.00, 0.00, mm, 180);

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
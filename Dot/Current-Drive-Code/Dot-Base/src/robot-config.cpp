#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor RightMotorA = motor(PORT1, ratio6_1, true);
motor RightMotorB = motor(PORT4, ratio6_1, false);
motor_group Right = motor_group(RightMotorA, RightMotorB);
motor LeftMotorA = motor(PORT2, ratio6_1, false);
motor LeftMotorB = motor(PORT3, ratio6_1, true);
motor_group Left = motor_group(LeftMotorA, LeftMotorB);
rotation Left_Sensor = rotation(PORT7, true);
rotation Right_Sensor = rotation(PORT8, true);
motor Ring = motor(PORT5, ratio6_1, true);
motor Arm = motor(PORT6, ratio36_1, false);
digital_out Grabber = digital_out(Brain.ThreeWirePort.A);

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
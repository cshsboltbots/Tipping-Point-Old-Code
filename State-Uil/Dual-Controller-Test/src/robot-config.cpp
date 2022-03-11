#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor LeftMotorA = motor(PORT1, ratio18_1, true);
motor LeftMotorB = motor(PORT4, ratio18_1, false);
motor_group Left = motor_group(LeftMotorA, LeftMotorB);
motor RightMotorA = motor(PORT2, ratio18_1, false);
motor RightMotorB = motor(PORT3, ratio18_1, true);
motor_group Right = motor_group(RightMotorA, RightMotorB);
rotation Right_Sensor = rotation(PORT7, false);
rotation Left_Sensor = rotation(PORT8, true);
motor Ring = motor(PORT5, ratio6_1, true);
digital_out Grabber = digital_out(Brain.ThreeWirePort.A);
motor ArmMotorA = motor(PORT9, ratio18_1, true);
motor ArmMotorB = motor(PORT10, ratio18_1, false);
motor_group Arm = motor_group(ArmMotorA, ArmMotorB);
inertial Inertial = inertial(PORT6);
motor Red = motor(PORT11, ratio36_1, true);
controller Controller2 = controller(partner);

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
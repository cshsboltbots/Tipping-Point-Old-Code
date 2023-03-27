#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
controller Controller2 = controller(partner);
motor LeftMotorA = motor(PORT1, ratio18_1, true);
motor LeftMotorB = motor(PORT4, ratio18_1, true);
motor_group Left = motor_group(LeftMotorA, LeftMotorB);
motor RightMotorA = motor(PORT2, ratio18_1, false);
motor RightMotorB = motor(PORT5, ratio18_1, false);
motor_group Right = motor_group(RightMotorA, RightMotorB);
motor Ring = motor(PORT9, ratio6_1, false);
motor LiftMotorA = motor(PORT6, ratio36_1, true);
motor LiftMotorB = motor(PORT7, ratio36_1, false);
motor_group Lift = motor_group(LiftMotorA, LiftMotorB);
rotation Left_Sensor = rotation(PORT10, true);
rotation Right_Sensor = rotation(PORT13, true);
inertial Inertial = inertial(PORT12);
digital_out Single = digital_out(Brain.ThreeWirePort.A);
digital_out Double = digital_out(Brain.ThreeWirePort.B);

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
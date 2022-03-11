/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module: main.cpp                                                        */
/*    Author: 2344A                                                           */
/*    Descption: Dual-Controller-Test                                         */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Left                 motor_group   1, 4            
// Right                motor_group   2, 3            
// Right_Sensor         rotation      7               
// Left_Sensor          rotation      8               
// Ring                 motor         5               
// Grabber              digital_out   A               
// Arm                  motor_group   9, 10           
// Inertial             inertial      6               
// Red                  motor         11              
// Controller2          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

// Includes the vex library
#include "vex.h"

// Uses the vex namespace
using namespace vex;

// A global instance of competition
competition Competition;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
}

// Base Code For PID from https://www.vexforum.com/t/vexcode-pid-tutorial/73706
// and was modified heavily for efficiency and use case.

// Constants
double kPR;
double kIR;
double kDR;
double kPL;
double kIL;
double kDL;

// Auton
int desiredValueright = 0;
int desiredValueleft = 0;

int leftError;          // Sensor value - Desiored Value : Position
int leftPrevError = 0;  // Position 20 msec ago
int leftDerivative;     // error - prevError : Speed
int leftTotalError = 0; // totalError = totalError + error

int rightError;          // Sensor value - Desiored Value : Position
int rightPrevError = 0;  // Position 20 msec ago
int rightDerivative;     // error - prevError : Speed
int rightTotalError = 0; // totalError = totalError + error

// Reset drive sensors
bool resetDriveSensors = false;

// global variable to control PID

bool enableDrivePID = true;

int drivePID() {

  // Only run while when PID is true.
  while (enableDrivePID) {

    if (resetDriveSensors) {
      resetDriveSensors = false;
      Left_Sensor.resetPosition();
      Right_Sensor.resetPosition();
    }

    // Get the position of the Motors
    int leftMotorPosition = Left_Sensor.position(degrees);
    int rightMotorPosition = Right_Sensor.position(degrees);

    /////////////////////////////
    // Left Lateral Movement PID//
    /////////////////////////////

    // Potential
    leftError = desiredValueleft - leftMotorPosition;

    // Derivative
    leftDerivative = leftError - leftPrevError;

    // Integral
    leftTotalError += leftError;

    double leftLateralMotorPower =
        (leftError * kPL + leftDerivative * kDL + leftTotalError * kIL);

    //////////////////////////////
    // Right Lateral Movement PID//
    //////////////////////////////

    // Potential
    rightError = desiredValueright - rightMotorPosition;

    // Derivative
    rightDerivative = rightError - rightPrevError;

    // Integral
    rightTotalError += rightError;

    double rightLateralMotorPower =
        (rightError * kPR + rightDerivative * kDR + rightTotalError * kIR);

    ///////////////////
    // Drive and Reset//
    ///////////////////

    Left.spin(forward, leftLateralMotorPower, voltageUnits::volt);
    Right.spin(forward, rightLateralMotorPower, voltageUnits::volt);

    leftPrevError = leftError;
    rightPrevError = rightError;
    wait(20, msec);
  }
  return 1;
}

bool enableRampBalance = true;

// This function when called initializes a pD loop for ramp balancing going in
// the forward direction.
int RampBalanceForward() {
  double cpitch = Inertial.roll();
  while (enableRampBalance) {
    cpitch = Inertial.roll();
    if (cpitch > 23) {
      Right.spin(reverse, 25, pct);
      Left.spin(reverse, 25, pct);
    } else if (cpitch < -22) {
      Right.spin(forward, 25, pct);
      Left.spin(forward, 25, pct);
    } else if (((cpitch < 22 && cpitch > -22) || cpitch > .5)) {
      Right.stop(hold);
      Left.stop(hold);
    }
    Brain.Screen.print(cpitch);
    wait(20, msec);
  }
  return 1;
}

// This function when called initializes a pD loop for ramp balancing going in
// the backward direction.
int RampBalanceBackward() {
  double cpitch = Inertial.roll();
  while (enableRampBalance) {
    cpitch = Inertial.roll();
    if (cpitch > -23) {
      Right.spin(reverse, 25, pct);
      Left.spin(reverse, 25, pct);
    } else if (cpitch < 22) {
      Right.spin(forward, 25, pct);
      Left.spin(forward, 25, pct);
    } else if (((cpitch < -22 && cpitch > 22) || cpitch > .5)) {
      Right.stop(hold);
      Left.stop(hold);
    }
    Brain.Screen.print(cpitch);
    wait(20, msec);
  }
  return 1;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

// Variable to store wheel circumference (in inches)
double wheelSize = 10.21018;

// Functions below make driving the robot much easier (user abstraction)

// This function converts the wheel size to degrees
// double input - wheel size in inches
int Convert(double input) { return ((input * 360) / wheelSize); }

// Function to Interact with Dirve Train
// double valueL - inches to the left (+ for forwards) and (- for backwards)
// double valueR - inches to the right (+ for forwards) and (- for backwards)
int Drive(double valueL, double valueR) {
  double valueDegreeL = Convert(valueL);
  double valueDegreeR = Convert(valueR);
  resetDriveSensors = true;
  if (valueL * valueR > 0) {
    kPR = 0.02;
    kIR = 0.000011;
    kDR = 0.04;
    kPL = 0.0188;
    kIL = 0.000011;
    kDL = 0.04;
  } else if (valueL * valueR < 0) {
    kPR = 0.02175;
    kIR = 0.00001;
    kDR = 0.005;
    kPL = 0.02175;
    kIL = 0.00001;
    kDL = 0.005;
  }
  desiredValueright = valueDegreeR;
  desiredValueleft = valueDegreeL;
  wait(1000, msec);
  return 1;
}

// Easy way to control ring lift
// double time - run for time seconds
// int dir - (1) for forward / (-1) for reverse
// int speed - speed to run in pct
int RunRing(double time, int dir, int speed) {
  if (dir == 1) {
    Ring.spin(reverse, speed, pct);
  } else {
    Ring.spin(forward, speed, pct);
  }
  wait(time, sec);
  Ring.stop();
  return 1;
}

// Easy way to control lift
// double time - run for time seconds
// int dir - (1) for forward / (-1) for reverse
// int speed - speed to run in pct
int RunLift(double time, int dir, int speed) {
  if (dir == 1) {
    Arm.spin(reverse, speed, pct);
  } else {
    Arm.spin(forward, speed, pct);
  }
  wait(time, sec);
  return 1;
}

// Easy way to control ring lift
// double revolutions
// int dir - (1) for forward / (-1) for reverse
int RunRingT(double revs, int dir) {
  if (dir == 1) {
    Ring.setVelocity(100, pct);
    Ring.spinToPosition(-1 * revs, rev, false);
  } else {
    Ring.setVelocity(100, pct);
    Ring.spinToPosition(revs, rev, false);
  }
  return 1;
}

// Easy way to control lift
// double revolutions
// int dir - (1) for forward / (-1) for reverse
int RunLiftT(double revs, int dir) {
  if (dir == 1) {
    Arm.setVelocity(100, pct);
    Arm.spinToPosition(-1 * revs, rev, false);
  } else {
    Arm.setVelocity(100, pct);
    Arm.spinToPosition(revs, rev, false);
  }
  return 1;
}

// Easy way to control goal lift
// int postition - (0) lower | (1) raise
int RunGoal(int position) {
  if (position == 1) {
    Grabber.set(true);
  } else if (position == 0) {
    Grabber.set(false);
  }
  return 1;
}

// Easy way to control goal grabber
// int postition - (0) lower | (1) raise
int RunRed(int position) {
  if (position == 1) {
    Red.spinFor(.75, rev);
  } else if (position == 0) {
    Red.spinFor(-.75, rev);
  }
  return 1;
}

// Easy way to go forwards
// double amount - inches to go forward
int F(double amount) {
  Drive(amount, amount);
  return 1;
}

// Easy way to go backwards
// double amount - inches to go backward
int B(double amount) {
  Drive(-1 * amount, -1 * amount);
  return 1;
}

// Easy way to turn right
// double deg - the degree to turn
int R(double deg) {
  double amount = (.130889 * deg) + 1.6;
  Drive(amount, -1 * amount);
  return 1;
}

// Easy way to turn left
// double deg - the degree to turn
int L(double deg) {
  double amount = (.130889 * deg) + 1.6;
  Drive(-1 * amount, amount);
  return 1;
}

// Auton starts here
void autonomous(void) {
  // Start of Auton

  // Starts The Ring Lift
  RunRingT(100000, -1);

  // Starts PID task
  task PID(drivePID);

  // Go Forward 9 in.
  F(9);
  wait(.1, sec);

  // Run the Numatic Claw
  RunGoal(1);

  // Arc Turn
  Drive(-8, -38.5);
  wait(1, sec);

  // Go Backwards 80 in.
  B(80);
  wait(2, sec);

  // Start to Raise the Lift
  RunLiftT(2.5, 1);

  // Arc Turn
  Drive(.5, 56);
  wait(1, sec);

  // Go Forward 30 in.
  F(30);
  wait(.6, sec);

  // Let go of Numatic Claw
  RunGoal(0);

  // Go Backwards 16 in.
  B(16);
  wait(.4, sec);

  // Start to Lower the
  RunLiftT(2.5, -1);

  // Turn Left 72 deg.
  L(72);

  // Go Forwards 29 in.
  F(29);

  // Run the Numatic Claw
  RunGoal(1);
  wait(.4, sec);

  // Start to Raise the Lift
  RunLiftT(2.5, 1);

  // Go Backwards 72 in.
  B(72);

  // Turn Right 90 deg.
  R(90);

  // Go Forwards 33.5 in.
  F(33.5);

  // Let go of the Numatic Claw
  RunGoal(0);
  wait(.5, sec);

  // Go Backwards 6 in.
  B(6);

  // Turn Right 87 deg.
  R(87);

  // Go Forwards 54 in.
  F(54);
  wait(1.4, sec);

  // Turn Right 100 deg.
  R(100);

  // Start to Lower the Lift
  RunLiftT(2.5, -1);

  // Go Backwards 40 in.
  B(40);
  wait(.5, sec);

  // Go Forwards 10 in.
  F(10);

  // Turn Right 100 deg.
  R(100);

  // Go Fowards 15 in.
  F(15);

  // Run the Numatic Claw
  RunGoal(1);

  // Arc Turn
  Drive(-5, -32);
  wait(2, sec);

  // Go Backwards 80 in.
  B(80);
  wait(2.8, sec);

  // Run Motor Claw
  RunRed(1);

  // Turn Left 55 deg.
  L(55);

  // Go Backwards 40 in.
  B(40);

  // Turn Left 40 deg.
  L(40);

  // Manually Run Motor Claw
  Red.spin(forward);
  wait(2, sec);
  Red.stop();
  Red.setBrake(hold);
  wait(.5, sec);

  // Go Backwards 35 in.
  B(35);

  // Turn Left 48 deg.
  L(48);

  // Start to Raise the Lift
  RunLiftT(2.5, 1);
  wait(1, sec);

  // Go Forwards 16 in.
  F(16);

  // Start to Lower the Lift
  RunLiftT(2.5, -1);

  

  // Go Forwards 16 in.
  F(16);

  // Disable PID
  enableDrivePID = false;

  // Enable Ramp Balancing
  task Ramp(RampBalanceForward);

  // End of Auton
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

// Driver Control Thread
int Driver() {
  while (true) {
    if (Controller1.ButtonDown.pressing() == 1 &&
        Controller1.ButtonRight.pressing() == 1) {
      Left.stop(hold);
      Right.stop(hold);
    } else if (Controller1.ButtonDown.pressing() == 1 &&
               Controller1.ButtonLeft.pressing() == 1) {
      Left.stop(coast);
      Right.stop(coast);
    }     if (Controller2.ButtonDown.pressing() == 1 &&
        Controller2.ButtonRight.pressing() == 1) {
      Left.stop(hold);
      Right.stop(hold);
    } else if (Controller2.ButtonDown.pressing() == 1 &&
               Controller2.ButtonLeft.pressing() == 1) {
      Left.stop(coast);
      Right.stop(coast);
    }else {
      Left.spin(directionType::fwd,
                (Controller1.Axis3.value() + Controller1.Axis1.value() / 2),
                velocityUnits::pct);
      Right.spin(directionType::fwd,
                 (Controller1.Axis3.value() - Controller1.Axis1.value() / 2),
                 velocityUnits::pct);
    }
    wait(20, msec);
  }
}

// Ring Control Thread
int RingButtons() {
  while (true) {
    if (Controller1.ButtonL1.pressing() == 1) {
      Ring.spin(forward, 100, pct);
    } else if (Controller1.ButtonL2.pressing() == 1) {
      Ring.spin(reverse, 100, pct);
    } else {
      Ring.stop();
    }
    wait(20, msec);
  }
}

// Arm Control Thread
int ArmButtons() {
  while (true) {
    if (Controller1.ButtonR1.pressing() == 1) {
      Arm.spin(reverse, 100, pct);
    } else if (Controller1.ButtonR2.pressing() == 1) {
      Arm.spin(forward, 100, pct);
    } else {
      Arm.stop(hold);
    }
    wait(20, msec);
  }
}

// Grabber Control Thread
int GrabberButtons() {
  while (true) {
    if (Controller1.ButtonX.pressing() == 1) {
      Grabber.set(true);
    } else if (Controller1.ButtonA.pressing() == 1) {
      Grabber.set(false);
    }
    wait(20, msec);
  }
}

// Red Control Thread
int RedButtons() {
  while (true) {
    if (Controller1.ButtonY.pressing() == 1) {
      Red.spin(forward, 100, pct);
    } else if (Controller1.ButtonB.pressing() == 1) {
      Red.spin(reverse, 100, pct);
    } 
    else if (Controller2.ButtonR1.pressing() == 1) {
      Red.spin(forward, 20, pct);
    } else if (Controller2.ButtonR2.pressing() == 1) {
      Red.spin(reverse, 20, pct);
    } else {
      Red.stop(hold);
    }
    wait(20, msec);
  }
}

// User Control Function
void usercontrol(void) {

  // Disables PID at start of User Control
  enableDrivePID = false;

  // Disables RampBalance at start of User Control
  enableRampBalance = false;

  // Starts the threads for User Control Period
  task BaseControl(Driver);
  task ButtonControl1(RingButtons);
  task ButtonControl2(ArmButtons);
  task ButtonControl3(GrabberButtons);
  task ButtonControl4(RedButtons);
}

// Entry Point
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module: main.cpp                                                        */
/*    Author: 2344A                                                           */
/*    Descption: Q39                                                          */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Controller2          controller                    
// Left                 motor_group   1, 4            
// Right                motor_group   2, 5            
// Ring                 motor         9               
// Lift                 motor_group   6, 7            
// Left_Sensor          rotation      10              
// Right_Sensor         rotation      13              
// Inertial             inertial      12              
// Single               digital_out   A               
// Double               digital_out   B               
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

////////////
//PID Code//
////////////

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

//////////////////
//Ramp Balancing//
//////////////////

bool enableRampBalance = true;

// This function when called initializes a pD loop for ramp balancing going in
// the forward direction.
int RampBalanceForward() {
  double cpitch = Inertial.roll();
  while (enableRampBalance) {
    cpitch = Inertial.roll();
    if (cpitch < -18) {
      Right.spin(reverse, 25, pct);
      Left.spin(reverse, 25, pct);
    } else if (cpitch > 18) {
      Right.spin(forward, 25, pct);
      Left.spin(forward, 25, pct);
    } else  {
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

//////////////////////////
//Driving Base Functions//
//////////////////////////

// Variable to store wheel circumference (in inches)
double wheelSize = 10.4458*(7.0/5.0);

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
    kPR = 0.055; //was .02
    kIR = 0.000010; //was .000011
    kDR = 0.044;
    kPL = 0.0538; //was .0188
    kIL = 0.000010; //was .000011
    kDL = 0.044; //was .04
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

///////////////////////
//Ring Lift Functions//
///////////////////////

// Easy way to control Ring Lift
// double time - run for time seconds
// bool dir - true:forward  false:reverse
// double speed - speed to run lift
int RunRing(double time,bool dir,double speed){
  Ring.setVelocity(100,pct);
  if(dir){
    Ring.spin(forward);
  }
  else{
    Ring.spin(reverse);
  }
  wait(time,sec);
  Ring.stop();
  return 1;
}

// Easy way to control Ring Lift
// double revolutions
// bool dir - true:forward  false:reverse
int RunRing(double revs, bool dir) {
  if (dir) {
    Ring.setVelocity(100, pct);
    Ring.spinToPosition(revs, rev, false);
  } else {
    Ring.setVelocity(100, pct);
    Ring.spinToPosition(-1*revs, rev, false);
  }
  return 1;
}

//////////////////
//Lift Functions//
//////////////////

// Easy way to control Lift
// double time - run for time seconds
// bool dir - true:forward  false:reverse
// double speed - speed to run lift
int RunLift(double time, bool dir, double speed) {
  if (!dir) {
    Lift.spin(reverse, speed, pct);
  } else {
    Lift.spin(forward, speed, pct);
  }
  wait(time, sec);
  return 1;
}

// Easy way to control lift
// double revolutions
// bool dir - true:up  false:down
int RunLift(double revs, bool dir) {
  if (dir) {
    Lift.setVelocity(100, pct);
    Lift.spinToPosition(revs, rev, false);
  } else {
    Lift.setVelocity(100, pct);
    Lift.spinToPosition(-1*revs, rev, false);
  }
  return 1;
}

// Easy way to control Lift
// bool dir - true:up  false:down
int RunLift(bool dir) {
  if (dir) {
    Lift.setVelocity(100, pct);
    Lift.spinToPosition(2.5, rev, false);
  } else {
    Lift.setVelocity(100, pct);
    Lift.spinToPosition(-2.5, rev, false);
  }
  return 1;
}

//////////////////////
//Numatics Functions//
//////////////////////

// Easy way to control Lift Numatic
// bool grab - true:grab  false:ungrab
int RunSingle(bool grab){
  if(grab){
    Single.set(true);
  }else{
    Single.set(false);
  }
  return 1;
}

// Easy way to control Goal Numatic
// bool grab - true:grab  false:ungrab
int RunDouble(bool grab){
  if(grab){
    Double.set(false);
  }else{
    Double.set(true);
  }
  return 1;
}

////////////////////////
//Easy Drive Functions//
////////////////////////

// Easy way to go forwards
// double amount - inches to go forward
int F(double amount) {
  Drive(amount * 1.03226, amount * 1.03226);
  return 1;
}

// Easy way to go backwards
// double amount - inches to go backward
int B(double amount) {
  Drive(-1 * amount * 1.03226, -1 * amount * 1.003226);
  return 1;
}

// Easy way to turn right
// double deg - the degree to turn
int R(double deg) {
  double amount = (-.0000460976)*((deg)*(deg)) + ((.141975)*deg) + 2.47131;
  Drive(amount, -1 * amount);
  return 1;
}

// Easy way to turn left
// double deg - the degree to turn
int L(double deg) {
  double amount = (-.0000460976)*((deg)*(deg)) + ((.141975)*deg) + 2.47131;  Drive(-1 * amount,amount);
  return 1;
}

/////////////////////
//Auton Starts Here//
/////////////////////

void autonomous(void) {

  // Start PID
  task PID(drivePID);

  // Open Double
  RunDouble(false);

  // Forward 48in.
  F(48);

  // Grab Goal with Double
  RunDouble(true);

  // Raise Lift
  RunLift(.2,true);

  // Backward 50in.
  B(50);
  wait(.5,sec);

  // 95 deg Right Turn
  R(95);
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
    if (Controller1.ButtonL2.pressing() == 1) {
      Ring.spin(reverse, 100, pct);
    } else if (Controller1.ButtonL1.pressing() == 1) {
      Ring.spin(forward, 100, pct);
    } else {
      Ring.stop();
    }
    wait(20, msec);
  }
}

// Lift Control Thread
int LiftButtons() {
  while (true) {
    if (Controller1.ButtonR1.pressing() == 1) {
      Lift.spin(forward, 100, pct);
    } else if (Controller1.ButtonR2.pressing() == 1) {
      Lift.spin(reverse, 100, pct);
    } else {
      Lift.stop(hold);
    }
    wait(20, msec);
  }
}

// Single Grabber Control Thread
int SingleButtons() {
  while (true) {
    if (Controller1.ButtonY.pressing() == 1) {
      Single.set(true);
    } else if (Controller1.ButtonB.pressing() == 1) {
      Single.set(false);
    }
    wait(20, msec);
  }
}

// Double Grabber Control Thread
int DoubleButtons() {
  while (true) {
    if (Controller1.ButtonX.pressing() == 1) {
      Double.set(false);
    } else if (Controller1.ButtonA.pressing() == 1) {
      Double.set(true);
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

  // Starts the threads for User Control
  task BaseControl(Driver);
  task ButtonControl1(RingButtons);
  task ButtonControl2(LiftButtons);
  task ButtonControl3(SingleButtons);
  task ButtonControl4(DoubleButtons);
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
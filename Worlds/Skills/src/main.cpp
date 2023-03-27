/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module: main.cpp                                                        */
/*    Author: 2344A                                                           */
/*    Descption: Skills                                                       */
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
    if (cpitch < -19) {
      Right.spin(reverse, 25, pct);
      Left.spin(reverse, 25, pct);
    } else if (cpitch > 18) {
      Right.spin(forward, 25, pct);
      Left.spin(forward, 25, pct);
    } else {
      Right.stop(hold);
      Left.stop(hold);
    }
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
  Ring.setVelocity(speed,pct);
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
    Lift.spinToPosition(2.3, rev, false);
  } else {
    Lift.setVelocity(100, pct);
    Lift.spinToPosition(-2.3, rev, false);
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

// Starts The Ring Lift
RunRing(100000, -1);
// Go Forward 8 in.
F(8);
wait(.1, sec);
// Run the Numatic Claw
RunDouble(true);
wait(.2,sec);
// Arc Turn
Drive(-8.5, -42); //was 38.5
wait(1, sec);
// Go Backwards 80 in.
B(80);
wait(2, sec);
// Start to Raise the Lift
RunLift(true);
// Arc Turn
Drive(1.5, 54); //.5 and 56
wait(1, sec);
// Go Forward 30 in.
F(30);
wait(.6, sec);
// Let go of Numatic Claw
RunDouble(false);
wait(.5,sec);
// Go Backwards 16 in.
B(16);
wait(.4, sec);
// Start to Lower the Lift
RunLift(false);
// Turn Left 72 deg.
L(72);
// Go Forwards 24 in.
F(24);
// Run the Numatic Claw
RunDouble(true);
wait(.4, sec);
// Start to Raise the Lift
RunLift(true);
// Go Backwards 72 in.
B(72);
// Turn Right 90 deg.
R(90);
// Go Forwards 31 in.
F(31);
// Let go of the Numatic Claw
RunDouble(false);
wait(.5, sec);
// Go Backwards 78 in.
B(78);
wait(1.5,sec);

// Turn Left 45 deg.
L(45);

// Go Backwards 68in.
B(68);
wait(.5,sec);

// Tunr right 45 deg
R(45);

// Go Forwards 20in.
F(20);

// Grab Goal
RunDouble(true);

// Do 150 right turn
R(150);

// Go Backwards 30in.
B(30);

// Stop Ring Lift
Ring.stop();

// Grab Single
RunSingle(true);

// DO a 180
L(180);

RunLift(true);

// Go Forwards 40in.
F(40);

// Unclip goal
RunDouble(false);

// Turn 45 deg left
L(50);

// Lower lift
RunLift(false);

// Forwards 64in.
F(64);

// Grab Goal
RunDouble(true);

// Go Backwards 10in.
B(10);

// Left 90 deg
L(90);

// Go Forwards 118in.
F(118);

// Arc Turn to get on ramp
Drive(2,15);

// Drive forward 10in.
F(10);

// Disable PID
enableDrivePID = false;

// Ramp Balance
task Ramp(RampBalanceForward);

// 
/*
// Go Forwards 54 in.
F(54);
wait(1.4, sec);
// Turn Right 100 deg.
R(100);
// Start to Lower the Lift
RunLift(2.5, -1);
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
RunDouble(true);
// Arc Turn
Drive(-5, -34.5);
wait(2, sec);
// Go Backwards 80 in.
B(80);
wait(2.8, sec);
// Manually Run Motor Claw
RunSingle(true);
wait(.5, sec);
// Turn Left 80 deg.
L(80);
// Go Backwards 40 in.
B(40);
wait(2,sec);
// Turn Left 40 deg.
Drive(8,21);
// Manually Run Motor Claw
RunSingle(false);
wait(.5, sec);
// Start to Raise the Lift
RunLift(2.5, 1);
wait(2, sec);
// Go Forwards 16 in.
F(16);
// Start to Lower the Lift
RunLift(2.5,-1);
wait(1,sec);
// Go Forwards 55 in.
F(55);
// Disable PID
enableDrivePID = false;
// Enable Ramp Balancing
task Ramp(RampBalanceForward);
// End of Auton


  // Forward 8in.
  F(8);

  // Grab Goal with Double
  RunDouble(true);

  // wait
  wait(.5,sec);

  // Raise Lift
  RunLift(.2,true);

  // Backward 20in.
  B(10);

  // 90 deg Left Turn
  L(90);

  // Forward 20in.
  F(20);

  // 109 deg Left Turn
  L(109);

  // Backward 100
  B(100);
  
  // wait
  wait(1.6,sec);

  // Grab goal
  RunSingle(true);

  // Forward 14in.
  F(14);

  // 60 deg Right Turn
  R(60);

  // Start to Raise Lift
  RunLift(true);

  // Run Ring Lift for Rest of Match
  RunRing(10000,true,100);

  // Forward 84in.
  F(84);



  // Let go of Goal
  RunDouble(false);

  // Backward 15in.
  B(15);

  // Start to Lower Lift
  RunLift(false);

  // 180 deg Left Turn
  L(180);

  // Forward 20in.
  F(20);

  // Grab Goal with Double
  RunDouble(true);

  // Start to Raise Lift
  RunLift(true);

  // 170 deg Left Turn
  L(170);

  // Forward 30in.
  F(30);

  // Drop Goal
  RunDouble(false);

  // 135 deg Left Turn
  L(135);

  // Start to Lower Lift
  RunLift(false);

  // Forward 60in.
  F(60);

  // Grab Goal
  RunDouble(true);

  // Start to Raise Lift
  RunLift(true);

  // 170 deg Left Turn
  L(170);

  // Forward 60in.
  F(60);

  // Drop Goal
  RunDouble(false);

  // 90 deg Turn Left
  L(90);

  // Lets go of other goal
  RunSingle(false);

  // Forward 10in.
  F(10);

  // 180 deg Turn Right
  R(180);

  // Forward 15in.
  F(15);

  // Grab Goal
  RunDouble(true);

  // Start to Raise Lift
  RunLift(true);

  // 100 deg Turn Right
  R(100);

  // Forward 15in.
  F(15);

  // Drop Goal
  RunDouble(false);

  // Backward 15in.
  B(15);

  // 80 deg Turn Right
  R(80);

  // Backward 40in.
  B(40);

  // Grab Goal
  RunSingle(true);

  // 30 deg Right Turn
  R(30);

  // Start to Lower Lift
  RunLift(false); 

  // Forward 90in.
  F(90);

  // Grab Goal
  RunDouble(true);

  // Start to Raise Lift
  RunLift(true);

  // 135 deg Turn Left
  L(135);

  // Forward 50in.
  F(50);

  // Drop Goal
  RunDouble(false);

  // Backward 10in.
  B(10);

  // 135 deg Turn Right
  R(135);

  //Start to Lower Lift
  RunLift(false);

  // Forward 10in.
  F(10);

  // 90 deg Left Turn
  L(90);

  // Forward 10in.
  F(10);

  // Grab Goal
  RunDouble(true);

  // Backward 50in.
  B(50);

  // Start to Raise Lift
  RunLift(true);

  // 180 deg Left Turn
  L(180);

  // Forward 50in.
  F(50);

  // Arc Turn
  Drive(21,3);

  // Start to Lower Lift
  RunLift(false);

  // Forward 10in.
  F(10);

  // Disable PID
  enableDrivePID = false;

  // Enable Ramp Balancing
  task Ramp(RampBalanceForward);
  */
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
    if (Controller1.ButtonX.pressing() == 1) {
      Single.set(true);
    } else if (Controller1.ButtonA.pressing() == 1) {
      Single.set(false);
    }
    wait(20, msec);
  }
}

// Double Grabber Control Thread
int DoubleButtons() {
  while (true) {
    if (Controller1.ButtonY.pressing() == 1) {
      Double.set(false);
    } else if (Controller1.ButtonB.pressing() == 1) {
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
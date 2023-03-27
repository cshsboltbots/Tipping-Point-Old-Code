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
// Right                motor_group   1, 4            
// Left                 motor_group   2, 3            
// Left_Sensor          rotation      7               
// Right_Sensor         rotation      8               
// Ring                 motor         5               
// Arm                  motor         6               
// Grabber              digital_out   A               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

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
  
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

// PID Video for Reference https://www.vexforum.com/t/vexcode-pid-tutorial/73706

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

int leftError; // Sensor value - Desiored Value : Position
int leftPrevError = 0; // Position 20 msec ago
int leftDerivative; // error - prevError : Speed
int leftTotalError = 0; // totalError = totalError + error

int rightError; // Sensor value - Desiored Value : Position
int rightPrevError = 0; // Position 20 msec ago
int rightDerivative; // error - prevError : Speed
int rightTotalError = 0; // totalError = totalError + error

// Reset drive sensors
bool resetDriveSensors = false;

// global variable to control PID

bool enableDrivePID = true;

int drivePID(){

  // Only run while when PID is true.
  while(enableDrivePID) {

    if (resetDriveSensors) {
      resetDriveSensors = false;
      Left_Sensor.resetPosition();
      Right_Sensor.resetPosition();
    }

    // Get the position of the Motors
    int leftMotorPosition = Left_Sensor.position(degrees);
    int rightMotorPosition = Right_Sensor.position(degrees);   

    /////////////////////////////
    //Left Lateral Movement PID//
    /////////////////////////////

    // Potential
    leftError = desiredValueleft - leftMotorPosition;

    // Derivative
    leftDerivative = leftError - leftPrevError;
    
    // Integral
    leftTotalError += leftError;

    double leftLateralMotorPower = (leftError * kPL + leftDerivative * kDL + leftTotalError * kIL);
    
    //////////////////////////////
    //Right Lateral Movement PID//
    //////////////////////////////

    // Potential
    rightError = desiredValueright - rightMotorPosition;

    // Derivative
    rightDerivative = rightError - rightPrevError;
    
    // Integral
    rightTotalError += rightError;

    double rightLateralMotorPower = (rightError * kPR + rightDerivative * kDR + rightTotalError * kIR);

    ///////////////////
    //Drive and Reset//
    ///////////////////

    Left.spin(forward, leftLateralMotorPower, voltageUnits::volt);
    Right.spin(forward, rightLateralMotorPower, voltageUnits::volt);

    leftPrevError = leftError;
    rightPrevError = rightError;
    wait(20,msec);

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

// Functions that make driving the robot much easier

// This function converts the wheen size to degrees
int Convert(double input) {
  return ((input*360)/wheelSize);
}

//This function drivesS
int Drive(double valueL,double valueR){
  double valueDegreeL = Convert(valueL);
  double valueDegreeR = Convert(valueR);
  resetDriveSensors = true;
  if (valueL * valueR > 0) {
    kPR = 0.020;
    kIR = 0.000011;
    kDR = 0.04;
    kPL = 0.0188;
    kIL = 0.000011;
    kDL = 0.04;
  }
  else if (valueL * valueR < 0) {
    kPR = 0.02175;
    kIR = 0.00001;
    kDR = 0.005;
    kPL = 0.02175;
    kIL = 0.00001;
    kDL = 0.005;
  }
  desiredValueright = valueDegreeR;
  desiredValueleft= valueDegreeL;
  wait(1000,msec);
  return 1;
}

//Easy way to control ring lift
//double rotations - rotations
//int dir - (1) for forward / (-1) for reverse
void RunRing(double rotations,int dir){
  Ring.setVelocity(100,pct);
  if (dir == 1){
    Ring.startSpinFor(reverse, rotations,rev);
  }
  else {
    Ring.startSpinFor(forward, rotations,rev);
  }
}

//Easy way to control goal lift
//int postition - (0) lower | (1) raise
void RunGoal(int position){
  if(position==1){
    Grabber.set(true);
  }
  else if(position==0){
    Grabber.set(false);
  }
}

int Multi1(){
    wait(.98,sec);
    Grabber.set(true);
    wait(.15,sec);
    RunRing(10000,-1);
  return 1;
}

// Auton starts here
void autonomous(void) {
  // Starts PID task
  vex::task PID(drivePID);
  vex::task M1(Multi1);
  //Angle Constant
  double angle = 0.131362;
  double drive = 1.009;
  //First Drive Forward
  Drive(86*drive,86*drive);
  wait(2.5,sec);
  RunGoal(0);
  //First Turn
  Drive(60*angle,-60*angle);
  //Fist NGoal
  Drive(-73.3*drive,-73.3*drive);
  wait(2,sec);
  RunGoal(1);
  //Turn to second NGoal
  Drive(-85*angle,85*angle);
  //Drive to Second NGoal
  Drive(61*drive,61*drive);
  wait(1,sec);
  //Turn third
  Drive(86*angle,-86*angle);
  //Drive third
  Drive(-70*drive,-70*drive);
  wait(1.3,sec);
  //Turn to red goal
  Drive(-63.8*angle,63.8*angle);
  RunGoal(0);
  //Drive to Red Goal
  Drive(20*drive,20*drive);
  Drive(180*angle,-180*angle);
  Drive(-34*drive,-34*drive);
  Drive(-180*angle,180*angle);
  Drive(47*drive,47*drive);
  wait(.7,sec);
  RunGoal(1);
  //Turn to move goal
  Drive(10.5*angle,-10.5*angle);
  //move to opposite side
  Drive(-128*drive,-128*drive);
  wait(2.4,sec);
  //turn for ramp
  Drive(23.75*drive,23.75*drive);
  //drive up ramp
  Drive(-60*angle,60*angle);
  Drive(-12*drive,-12*drive);
  Drive(-62.4*angle,62.4*angle);
  Drive(-38*drive,-38*drive);
  Ring.stop();
  //END
  //END
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
int Drive(){
  while (true){
    //////////////////
    //Driver Control//
    //////////////////
    if(Controller1.ButtonDown.pressing() == 1 && Controller1.ButtonRight.pressing() == 1){
      Left.stop(hold);
      Right.stop(hold);
    }
    else if(Controller1.ButtonDown.pressing() == 1 && Controller1.ButtonLeft.pressing() == 1){
      Left.stop(coast);
      Right.stop(coast);
    }
    else{
    Left.spin(vex::directionType::fwd, (Controller1.Axis3.value()*2 + Controller1.Axis1.value())/2, vex::velocityUnits::pct);
    Right.spin(vex::directionType::fwd, (Controller1.Axis3.value()*2 - Controller1.Axis1.value())/2, vex::velocityUnits::pct);
    }
    wait(20, msec);
  }
}

int Button(){
  while(true){
    if (Controller1.ButtonL1.pressing() == 1) {
      Ring.spin(forward,100,pct);
    }
    else if (Controller1.ButtonL2.pressing() == 1) {
      Ring.spin(reverse,100,pct);
    }
    else if (Controller1.ButtonR1.pressing() == 1) {
      Arm.spin(reverse,50,pct);
    }
    else if (Controller1.ButtonR2.pressing() == 1) {
      Arm.spin(forward,50,pct);
    }
    else if (Controller1.ButtonY.pressing() == 1) {
      //Extend
      Grabber.set(true);
    }
    else if (Controller1.ButtonB.pressing() == 1) {
      Grabber.set(false);
    }
    else {
      Ring.stop();
      Arm.stop(hold);
    }
    wait(20, msec);
  }
}

void usercontrol(void) {
// Disables PID at start of User Control
enableDrivePID = false;
// User control code here, inside the loop
vex::task BaseControl(Drive);
vex::task ButtonControl(Button);
}

//
// Main will set up the competition functions and callbacks.
//
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
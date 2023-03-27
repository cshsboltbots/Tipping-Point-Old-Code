/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module: main.cpp                                                        */
/*    Author: 2344A                                                           */
/*    Descption: Green-Dot-Code                                               */
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

// PID Video https://www.vexforum.com/t/vexcode-pid-tutorial/73706

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
//double time - run for time seconds
//int dir - (1) for forward / (-1) for reverse
//int speed - speed to run in pct
int RunRing(double time,int dir,int speed){
  if (dir == 1){
    Ring.spin(reverse,speed,pct);
  }
  else {
    Ring.spin(forward,speed,pct);
  }
  wait(time,sec);
  Ring.stop();
  return 1;
}

//Easy way to control goal lift
//int postition - (0) lower | (1) raise
int RunGoal(int position){
  if(position==1){
    Grabber.set(true);
  }
  else if(position==0){
    Grabber.set(false);
  }
  return 1;
}

// Auton starts here
void autonomous(void) {
  // Starts PID task
  vex::task PID(drivePID);
  //Code for Auton 
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
    Left.spin(vex::directionType::fwd, (Controller1.Axis3.value()*1.5 + Controller1.Axis1.value()/3), vex::velocityUnits::pct);
    Right.spin(vex::directionType::fwd, (Controller1.Axis3.value()*1.5 - Controller1.Axis1.value()/3), vex::velocityUnits::pct);
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
    else if (Controller1.ButtonY.pressing() == 1){
      //Extend
      Grabber.set(true);
    }
    else if (Controller1.ButtonB.pressing() == 1){
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
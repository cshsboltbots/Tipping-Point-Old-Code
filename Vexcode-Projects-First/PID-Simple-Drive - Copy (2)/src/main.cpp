/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  PID-Simple-Drive                                       */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Left                 motor_group   1, 4            
// Right                motor_group   2, 3            
// Right_Sensor         rotation      11              
// Left_Sensor          rotation      12              
// Inertial             inertial      13              
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
double kPR = 0.0072;
double kIR = 0.00001;
double kDR = 0.0005;
double kPL = 0.006034;
double kIL = 0.00001;
double kDL = 0.0008355;
double turnkP = 0;
double turnkI = 0;
double turnkD = 0;

// Auton
int desiredValue = 0;
int desiredValueright = 0;
int desiredValueleft = 0;
int desiredTurnValue = 0;

int leftError; // Sensor value - Desiored Value : Position
int leftPrevError = 0; // Position 20 msec ago
int leftDerivative; // error - prevError : Speed
int leftTotalError = 0; // totalError = totalError + error

int rightError; // Sensor value - Desiored Value : Position
int rightPrevError = 0; // Position 20 msec ago
int rightDerivative; // error - prevError : Speed
int rightTotalError = 0; // totalError = totalError + error

int leftTurnError; // Sensor value - Desiored Value : Position
int leftTurnPrevError = 0; // Position 20 msec ago
int leftTurnDerivative; // error - prevError : Speed
int leftTurnTotalError = 0; // totalError = totalError + error

int rightTurnError; // Sensor value - Desiored Value : Position
int rightTurnPrevError = 0; // Position 20 msec ago
int rightTurnDerivative; // error - prevError : Speed
int rightTurnTotalError = 0; // totalError = totalError + error

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

    /////////////////////////////
    //Left Turning Movement PID//
    /////////////////////////////
    /*
    // Potential
    leftTurnError = desiredTurnValue - leftMotorPosition;

    // Derivative
    leftTurnDerivative = leftTurnError - leftTurnPrevError;
    
    // Integral
    leftTurnTotalError += leftTurnError;

    double leftTurnMotorPower = (leftTurnError * turnkP + leftTurnDerivative * turnkD + leftTurnTotalError * turnkI);

    //////////////////////////////
    //Right Turning Movement PID//
    //////////////////////////////

    // Potential
    rightTurnError = desiredTurnValue - rightMotorPosition;

    // Derivative
    rightTurnDerivative = rightTurnError - rightTurnPrevError;
    
    // Integral
    rightTurnTotalError += rightTurnError;

    double rightTurnMotorPower = (rightTurnError * turnkP + rightTurnDerivative * turnkD + rightTurnTotalError * turnkI);
    */
    ///////////////////
    //Drive and Reset//
    ///////////////////

    Left.spin(forward, leftLateralMotorPower, voltageUnits::volt);
    Right.spin(forward, rightLateralMotorPower, voltageUnits::volt);

    leftPrevError = leftError;
    rightPrevError = rightError;
    leftTurnPrevError = leftTurnError;
    rightTurnPrevError = rightTurnError;
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
double wheelSize = 12.56;

// Functions that make driving the robot much easier

// This function converts the wheen size to degrees
int ConvertInch(double input) {
  return ((input*360)/wheelSize);
}

int F(double valueR,double valueL){
  double valueDegreeR = ConvertInch(valueR);
  double valueDegreeL = ConvertInch(valueL);
  resetDriveSensors = true;
  desiredValueright = valueDegreeR;
  desiredValueleft= valueDegreeL;
  wait(1000, msec);
  return 1;
}

int Turning(double value){
  resetDriveSensors = true;
  desiredTurnValue = value;
  wait(1000,msec);
  return 1;
}

// Auton starts here
void autonomous(void) {
  // Starts PID task
  vex::task PID(drivePID);

  //Brain.Screen.print("PID Started");

  // Auton Commands
  //Left.spinFor(forward,.5,sec);
  //Right.spinFor(forward,.5,sec);
  F(40,40);
  //Brain.Screen.clearScreen();
  //Brain.Screen.print("PID Done");
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

void usercontrol(void) {
  // User control code here, inside the loop
  
  // Disables PID at start of User Control

  bool enableDrivePID = true;

  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.
    
    //////////////////////////////
    //User Controler Programming//
    //////////////////////////////

    // User Control Scheme: Split Arcade
    
    // See video https://www.youtube.com/watch?v=WrF0evf97xs

    if (Controller1.ButtonR1.pressing() == 1){
      if (Controller1.Axis3.position(percent) !=0){
        Left.spin(forward, Controller1.Axis3.position(),percent);
        Right.spin(forward, Controller1.Axis3.position(),percent);
      }
      else if (Controller1.Axis1.position(percent) !=0){
        Left.spin(forward, Controller1.Axis1.position(),percent);
        Right.spin(reverse, Controller1.Axis1.position(),percent);
      }
    }
    else if (Controller1.Axis3.position(percent) !=0){
      Left.spin(forward, Controller1.Axis3.position()/2,percent);
      Right.spin(forward, Controller1.Axis3.position()/2,percent);
    }
    else if (Controller1.Axis1.position(percent) !=0){
      Left.spin(forward, Controller1.Axis1.position()/2,percent);
      Right.spin(reverse, Controller1.Axis1.position()/2,percent);
    }
    else {
      // Could be changed to break, coast, or hold depending on how it preforms.
      Left.stop(brake);
      Right.stop(brake);
    }

    //////////////////////////////
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  //Calibrates the GPS
  //GPS.calibrate();

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
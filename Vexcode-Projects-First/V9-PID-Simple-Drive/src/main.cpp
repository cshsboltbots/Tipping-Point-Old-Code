/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  V8-PID-Simple-Drive                                       */
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
double kP = 0.0;
double kI = 0.0;
double kD = 0.0;
double turnkP = 0.0;
double turnkI = 0.0;
double turnkD = 0.0;

// Auton
int desiredValue = 0;
int desiredTurnValue = 0;

int error; // Sensor value - Desiored Value : Position
int prevError = 0; // Position 20 msec ago
int derivative; // error - prevError : Speed
int totalError = 0; // totalError = totalError + error

int turnError; // Sensor value - Desiored Value : Position
int turnPrevError = 0; // Position 20 msec ago
int turnDerivative; // error - prevError : Speed
int turnTotalError = 0; // totalError = totalError + error

// Reset drive sensors
bool resetDriveSensors = false;

// global variable to control PID

bool enableDrivePID = false;

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
    int RightMotorPosition = Right_Sensor.position(degrees);   

    ////////////////////////
    //Lateral Movement PID//
    ////////////////////////

    // Get the average of the two motors
    int averagePosition = (leftMotorPosition + RightMotorPosition)/2;

    // Potential
    error = averagePosition -desiredValue;

    // Derivative
    derivative = error - prevError;
    
    // Integral
    totalError += error;

    double lateralMotorPower = (error * kP + derivative * kD + totalError * kI);
    
    ////////////////////////
    //Turning Movement PID//
    ////////////////////////

    int turnDifference = (leftMotorPosition - RightMotorPosition)/2;

    // Potential
    turnError = turnDifference -desiredTurnValue;

    // Derivative
    turnDerivative = turnError - turnPrevError;
    
    // Integral
    turnTotalError += turnError;

    double turnMotorPower = (turnError * turnkP + turnDerivative * turnkD + turnTotalError * turnkI);
    

    Left.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    Right.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt);

    prevError = error;
    turnPrevError = turnError;
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
int Convert(double input) {
  return ((input/360)*wheelSize);
}

int TR() {
  resetDriveSensors = true;
  desiredTurnValue = -90;
  waitUntil(Right.isDone() && Left.isDone());
  return 1;
}

int TL() {
  resetDriveSensors = true;
  desiredTurnValue = 90;
  waitUntil(Right.isDone() && Left.isDone());
  return 1;
}

int F(double value){
  value = Convert(value);
  resetDriveSensors = true;
  desiredTurnValue = value;
  waitUntil(Right.isDone() && Left.isDone());
  return 1;
}

int R(double value){
  value = Convert(value);
  resetDriveSensors = true;
  desiredTurnValue = value;
  waitUntil(Right.isDone() && Left.isDone());
  return 1;
}

// Auton starts here
void autonomous(void) {
  // Starts PID task
  vex::task PID(drivePID);

  // Auton Commands
  F(22.8);
  R(11.4);
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
    /*
    Use Control Scheme: Split Arcade
    
    // Option 1, See video https://www.youtube.com/watch?v=Q7fQ5S0KjuQ

    Left.setVelocity(Controller1.Axis1.position(),percent);
    Right.setVelocity(Controller1.Axis2.position(),percent);
    Left.spin(forward);
    Right.spin(forward);
    */

    // Option 2, See video https://www.youtube.com/watch?v=WrF0evf97xs

    if (Controller1.Axis3.position(percent) != 0 || Controller1.Axis1.position(percent) !=0){
      Left.spin(forward, Controller1.Axis3.position(),percent);
      Right.spin(forward, Controller1.Axis3.position(),percent);
    }
    else {
      // Could be changed to break, coast, or hold depending on how it preforms.
      Left.stop();
      Right.stop();
    }

    // Option 3, From prev year.
    /*
    Left.setBrake(coast);
    Right.setBrake(coast);
    Left.spin(vex::directionType::fwd, (Controller1.Axis3.value() + Controller1.Axis1.value()/2), vex::velocityUnits::pct);
    Right.spin(vex::directionType::fwd, (Controller1.Axis3.value() - Controller1.Axis1.value()/2), vex::velocityUnits::pct);
    */
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

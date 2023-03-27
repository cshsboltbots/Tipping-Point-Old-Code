/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  V1-PID-Simple-Drive                                       */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Left                 motor_group   1, 2            
// Right                motor_group   3, 4            
// Controller1          controller                    
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
      Left.setPosition(0,degrees);
      Right.setPosition(0,degrees);
    }

    // Get the position of the Motors
    int leftMotorPosition = Left.position(degrees);
    int RightMotorPosition = Right.position(degrees);   

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

// Functions that make driving the robot much easier

int TR() {
  resetDriveSensors = true;
  desiredTurnValue = -90;
  wait(1000,msec);
  return 1;
}

int TL() {
  resetDriveSensors = true;
  desiredTurnValue = 90;
  wait(1000,msec);
  return 1;
}

int F(int value){
  resetDriveSensors = true;
  desiredTurnValue = value;
  wait(1000,msec);
  return 1;
}

int R(int value){
  resetDriveSensors = true;
  desiredTurnValue = value;
  wait(1000,msec);
  return 1;
}

// Auton starts here
void autonomous(void) {
  // Starts PID task
  vex::task science(drivePID);
  F(300);
  R(150);
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
    
    // See video https://www.youtube.com/watch?v=Q7fQ5S0KjuQ

    Left.setVelocity(Controller1.Axis1.position(),percent);
    Right.setVelocity(Controller1.Axis2.position(),percent);
    Left.spin(forward);
    Right.spin(forward);
    

    // See video https://www.youtube.com/watch?v=WrF0evf97xs

    if (Controller1.Axis3.position(percent) != 0){
      Left.spin(forward, Controller1.Axis3.position(),percent);
      Right.spin(forward, Controller1.Axis3.position(),percent);
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

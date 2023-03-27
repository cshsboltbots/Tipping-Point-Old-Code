/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  New-PID-Drive-V1                                          */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Right_Sensor         rotation      11              
// Left_Sensor          rotation      12              
// Inertial             inertial      13              
// Left                 motor_group   1, 4            
// Right                motor_group   2, 3            
// GPS                  gps           14              
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
// Auton starts here
void autonomous(void) {
  GPS.calibrate();
  Left.spin(forward);
  Right.spin(forward);
  Brain.Screen.yPosition();
  Brain.Screen.xPosition();
  waitUntil(GPS.xPosition()==0);
  Left.stop();
  Right.stop();
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

  while (true){
    Left.spin(vex::directionType::fwd, (Controller1.Axis3.value()*2 + Controller1.Axis1.value()/3), vex::velocityUnits::pct);
    Right.spin(vex::directionType::fwd, (Controller1.Axis3.value()*2 - Controller1.Axis1.value()/3), vex::velocityUnits::pct);
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.
    
    //////////////////////////////
    //User Controler Programming//
    //////////////////////////////

    // User Control Scheme: Split Arcade
    
    // See video https://www.youtube.com/watch?v=WrF0evf97xs
    /*if (Controller1.Axis1.position(percent) !=0 || Controller1.Axis3.position(percent) != 0) {
    Left.spin(forward, Controller1.Axis1.position(),percent);
    Right.spin(reverse, Controller1.Axis1.position(),percent);
    Left.spin(forward, Controller1.Axis3.position()/2,percent);
    Right.spin(forward, Controller1.Axis3.position()/2,percent);
    }
    else {
      // Could be changed to break, coast, or hold.
      Left.stop(brake);
      Right.stop(brake);
    }
    */
  //vex::task drivecontrol(driveTask);
  //dtask = vex::task(driveTask);
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
  GPS.calibrate();

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
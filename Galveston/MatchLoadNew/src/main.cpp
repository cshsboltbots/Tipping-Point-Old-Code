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
// Left                 motor_group   1, 4            
// Right                motor_group   2, 3            
// Right_Sensor         rotation      7               
// Left_Sensor          rotation      8               
// Ring                 motor         5               
// Grabber              digital_out   A               
// Arm                  motor_group   9, 10           
// Inertial             inertial      6               
// Red                  motor         11              
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
  Grabber.set(true);
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

bool enableRampBalance = true;

int RampBalanceForward(){
  double cpitch = Inertial.roll();
    while(enableRampBalance){
      cpitch = Inertial.roll()*-1;
      if(cpitch>1){
        while(cpitch>1){
        Right.spin(forward,10,pct);
        Left.spin(forward,10,pct);
        }
      }
      else if(cpitch<1){
        while(cpitch<1){
        Right.spin(reverse,10,pct);
        Left.spin(reverse,10,pct);
        }
      }
      Right.stop(hold);
      Left.stop(hold);
      wait(20,msec);
    }
  return 1;
}

int RampBalanceBackward(){
  double cpitch = Inertial.roll();
    while(enableRampBalance){
      cpitch = Inertial.roll();
      if(cpitch<3){
        Right.spin(reverse,10,pct);
        Left.spin(reverse,10,pct);
      }
      else if(cpitch>3){
        Right.spin(forward,10,pct);
        Left.spin(forward,10,pct);
      }
      Right.stop(hold);
      Left.stop(hold);
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
    kPR = 0.02;
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


//Easy way to control lift
//double time - run for time seconds
//int dir - (1) for forward / (-1) for reverse
//int speed - speed to run in pct
int RunLift(double time,int dir,int speed){
  if (dir == 1){
    Arm.spin(reverse,speed,pct);
  }
  else {
    Arm.spin(forward,speed,pct);
  }
  wait(time,sec);
  return 1;
}


//Easy way to control ring lift
//double revolutions
//int dir - (1) for forward / (-1) for reverse
int RunRingT(double revs,int dir){
  if (dir == 1){
    Ring.setVelocity(100,pct);
    Ring.spinToPosition(-1*revs, rev,false);
  }
  else {
    Ring.setVelocity(100,pct);
    Ring.spinToPosition(revs, rev,false);
  }
  return 1;
}


//Easy way to control lift
//double revolutions
//int dir - (1) for forward / (-1) for reverse
int RunLiftT(double revs,int dir){
  if (dir == 1){
    Arm.spinToPosition(-1*revs,rev,false);
  }
  else {
    Arm.spinToPosition(revs,rev,false);
  }
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

//Easy way to control goal grabber
//int postition - (0) lower | (1) raise
int RunRed(int position){
  if(position==1){
    Red.spinFor(.75,rev);
  }
  else if(position==0){
    Red.spinFor(-.75,rev);
  }
  return 1;
}

//Easy way to go forward
//double amount - the amount to go forward
int F(double amount){
  Drive(amount,amount);
  return 1;
}

//Easy way to go forward
//double amount - the amount to go forward
int B(double amount){
  Drive(-1*amount,-1*amount);
  return 1;
}

//Easy way to turn right
//double deg - the degree to turn
int R(double deg){
  double amount = (.130889*deg) + 1.6;
  Drive(amount,-1*amount);
  return 1;
}

//Easy way to turn left
//double deg - the degree to turn
int L(double deg){
  double amount = (.130889*deg) + 1.6;
  Drive(-1*amount,amount);
  return 1;
}

// Auton starts here
void autonomous(void) {
  // Starts PID task
  vex::task PID(drivePID);
  Grabber.set(false);
  RunRing(.5,-1,20);
  F(60);
  wait(.5,sec);
  RunGoal(1);
  B(40);
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
    Left.spin(vex::directionType::fwd, (Controller1.Axis3.value() + Controller1.Axis1.value()/2), vex::velocityUnits::pct);
    Right.spin(vex::directionType::fwd, (Controller1.Axis3.value() - Controller1.Axis1.value()/2), vex::velocityUnits::pct);
    }
    wait(20, msec);
  }
}

int RingButton(){
  while(true){
    if (Controller1.ButtonL1.pressing() == 1){
      Ring.spin(forward,100,pct);
    }
    else if (Controller1.ButtonL2.pressing() == 1){
      Ring.spin(reverse,100,pct);
    }
    else{
      Ring.stop();
    }
    wait(20, msec);
  }
}

int ArmButton(){
  while(true){
    if (Controller1.ButtonR1.pressing() == 1){
      Arm.spin(reverse,100,pct);
    }
    else if (Controller1.ButtonR2.pressing() == 1){
      Arm.spin(forward,100,pct);
    }
    else{
      Arm.stop(hold);
    }
    wait(20, msec);
  }
}

int GrabberButton(){
  while(true){
    if (Controller1.ButtonX.pressing() == 1){
      Grabber.set(true);
    }
    else if (Controller1.ButtonA.pressing() == 1){
      Grabber.set(false);
    }
    wait(20, msec);
  }
}

int RedButton(){
  while(true){
    if (Controller1.ButtonY.pressing() == 1){
      Red.spin(forward,100,pct);
    }
    else if (Controller1.ButtonB.pressing() == 1){
      Red.spin(reverse,100,pct);
    }
    else {
      Red.stop(hold);
    }
    wait(20,msec);
  }
}

void usercontrol(void) {
// Disables PID at start of User Control
enableDrivePID = false;
// Disables RampBalance
enableRampBalance = false;
// User control code here, inside the loop
vex::task BaseControl(Drive);
vex::task ButtonControl1(RingButton);
vex::task ButtonControl2(ArmButton);
vex::task ButtonControl3(GrabberButton);
vex::task ButtonControl4(RedButton);
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
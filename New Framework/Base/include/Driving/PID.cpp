#include "vex.h"
using namespace vex;

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
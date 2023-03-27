#include "PID.cpp"

// Function to Interact with Dirve Train
// double valueL - inches to the left (+ for forwards) and (- for backwards)
// double valueR - inches to the right (+ for forwards) and (- for backwards)

// Variable to store wheel circumference (in inches)
double wheelSize = 10.21018;

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
    kPR = 0.02;
    kIR = 0.000011;
    kDR = 0.04;
    kPL = 0.0188;
    kIL = 0.000011;
    kDL = 0.04;
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
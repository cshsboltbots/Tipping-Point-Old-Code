#include "Driving/Driving.cpp"

bool enableRampBalance = true;

// This function when called initializes a pD loop for ramp balancing going in
// the forward direction.
int RampBalanceForward() {
  double cpitch = Inertial.roll();
  while (enableRampBalance) {
    cpitch = Inertial.roll();
    if (cpitch > 23) {
      Right.spin(reverse, 25, pct);
      Left.spin(reverse, 25, pct);
    } else if (cpitch < -22) {
      Right.spin(forward, 25, pct);
      Left.spin(forward, 25, pct);
    } else if (((cpitch < 22 && cpitch > -22) || cpitch > .5)) {
      Right.stop(hold);
      Left.stop(hold);
    }
    Brain.Screen.print(cpitch);
    wait(20, msec);
  }
  return 1;
}

// This function when called initializes a pD loop for ramp balancing going in
// the backward direction.
int RampBalanceBackward() {
  double cpitch = Inertial.roll();
  while (enableRampBalance) {
    cpitch = Inertial.roll();
    if (cpitch > -23) {
      Right.spin(reverse, 25, pct);
      Left.spin(reverse, 25, pct);
    } else if (cpitch < 22) {
      Right.spin(forward, 25, pct);
      Left.spin(forward, 25, pct);
    } else if (((cpitch < -22 && cpitch > 22) || cpitch > .5)) {
      Right.stop(hold);
      Left.stop(hold);
    }
    Brain.Screen.print(cpitch);
    wait(20, msec);
  }
  return 1;
}
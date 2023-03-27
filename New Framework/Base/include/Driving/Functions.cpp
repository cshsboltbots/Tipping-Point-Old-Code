#include "PID_Tuning.cpp"

// Easy way to control ring lift
// double time - run for time seconds
// int dir - (1) for forward / (-1) for reverse
// int speed - speed to run in pct
int RunRing(double time, int dir, int speed) {
  if (dir == 1) {
    Ring.spin(reverse, speed, pct);
  } else {
    Ring.spin(forward, speed, pct);
  }
  wait(time, sec);
  Ring.stop();
  return 1;
}

// Easy way to control lift
// double time - run for time seconds
// int dir - (1) for forward / (-1) for reverse
// int speed - speed to run in pct
int RunLift(double time, int dir, int speed) {
  if (dir == 1) {
    Arm.spin(reverse, speed, pct);
  } else {
    Arm.spin(forward, speed, pct);
  }
  wait(time, sec);
  return 1;
}

// Easy way to control ring lift
// double revolutions
// int dir - (1) for forward / (-1) for reverse
int RunRingT(double revs, int dir) {
  if (dir == 1) {
    Ring.setVelocity(100, pct);
    Ring.spinToPosition(-1 * revs, rev, false);
  } else {
    Ring.setVelocity(100, pct);
    Ring.spinToPosition(revs, rev, false);
  }
  return 1;
}

// Easy way to control lift
// double revolutions
// int dir - (1) for forward / (-1) for reverse
int RunLiftT(double revs, int dir) {
  if (dir == 1) {
    Arm.setVelocity(100, pct);
    Arm.spinToPosition(-1 * revs, rev, false);
  } else {
    Arm.setVelocity(100, pct);
    Arm.spinToPosition(revs, rev, false);
  }
  return 1;
}

// Easy way to control goal lift
// int postition - (0) lower | (1) raise
int RunGoal(int position) {
  if (position == 1) {
    Grabber.set(true);
  } else if (position == 0) {
    Grabber.set(false);
  }
  return 1;
}

// Easy way to control goal grabber
// int postition - (0) lower | (1) raise
int RunRed(int position) {
  if (position == 1) {
    Red.spinFor(.75, rev);
  } else if (position == 0) {
    Red.spinFor(-.75, rev);
  }
  return 1;
}

// Easy way to go forwards
// double amount - inches to go forward
int F(double amount) {
  Drive(amount, amount);
  return 1;
}

// Easy way to go backwards
// double amount - inches to go backward
int B(double amount) {
  Drive(-1 * amount, -1 * amount);
  return 1;
}

// Easy way to turn right
// double deg - the degree to turn
int R(double deg) {
  double amount = (.130889 * deg) + 1.6;
  Drive(amount, -1 * amount);
  return 1;
}

// Easy way to turn left
// double deg - the degree to turn
int L(double deg) {
  double amount = (.130889 * deg) + 1.6;
  Drive(-1 * amount, amount);
  return 1;
}
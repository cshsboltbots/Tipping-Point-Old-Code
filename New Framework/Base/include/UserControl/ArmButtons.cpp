#include <vex.h>
using namespace vex;

// Arm Control Thread
int ArmButtons() {
  while (true) {
    if (Controller1.ButtonR1.pressing() == 1) {
      Arm.spin(reverse, 100, pct);
    } else if (Controller1.ButtonR2.pressing() == 1) {
      Arm.spin(forward, 100, pct);
    } else {
      Arm.stop(hold);
    }
    wait(20, msec);
  }
}
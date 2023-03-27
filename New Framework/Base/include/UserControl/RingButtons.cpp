#include <vex.h>
using namespace vex;

// Ring Control Thread
int RingButtons() {
  while (true) {
    if (Controller1.ButtonL1.pressing() == 1) {
      Ring.spin(forward, 100, pct);
    } else if (Controller1.ButtonL2.pressing() == 1) {
      Ring.spin(reverse, 100, pct);
    } else {
      Ring.stop();
    }
    wait(20, msec);
  }
}
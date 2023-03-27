#include <vex.h>
using namespace vex;

// Red Control Thread
int RedButtons() {
  while (true) {
    if (Controller1.ButtonY.pressing() == 1) {
      Red.spin(forward, 75, pct);
    } else if (Controller1.ButtonB.pressing() == 1) {
      Red.spin(reverse, 75, pct);
    } else {
      Red.stop(hold);
    }
    wait(20, msec);
  }
}
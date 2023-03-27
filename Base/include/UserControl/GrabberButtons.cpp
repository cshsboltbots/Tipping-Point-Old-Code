#include <vex.h>
using namespace vex;

// Grabber Control Thread
int GrabberButtons() {
  while (true) {
    if (Controller1.ButtonX.pressing() == 1) {
      Grabber.set(true);
    } else if (Controller1.ButtonA.pressing() == 1) {
      Grabber.set(false);
    }
    wait(20, msec);
  }
}
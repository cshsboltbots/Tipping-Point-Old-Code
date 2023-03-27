#include <vex.h>
using namespace vex;

// Driver Control Thread
int Driver() {
  while (true) {
    if (Controller1.ButtonDown.pressing() == 1 &&
        Controller1.ButtonRight.pressing() == 1) {
      Left.stop(hold);
      Right.stop(hold);
    } else if (Controller1.ButtonDown.pressing() == 1 &&
               Controller1.ButtonLeft.pressing() == 1) {
      Left.stop(coast);
      Right.stop(coast);
    } else {
      Left.spin(directionType::fwd,
                (Controller1.Axis3.value() + Controller1.Axis1.value() / 2),
                velocityUnits::pct);
      Right.spin(directionType::fwd,
                 (Controller1.Axis3.value() - Controller1.Axis1.value() / 2),
                 velocityUnits::pct);
    }
    wait(20, msec);
  }
}
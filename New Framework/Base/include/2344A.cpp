#include "RampBalancing/RampBalancing.cpp"
#include "UserControl/UserControl.cpp"
#include "PreAuton/PreAuton.cpp"
#include <vex.h>
using namespace vex;

// A global instance of competition
competition Competition;

// User Control Function
void usercontrol(void) {

  // Disables PID at start of User Control
  enableDrivePID = false;

  // Disables RampBalance at start of User Control
  enableRampBalance = false;

  // Starts the threads for User Control
  task BaseControl(Driver);
  task ButtonControl1(RingButtons);
  task ButtonControl2(ArmButtons);
  task ButtonControl3(GrabberButtons);
  task ButtonControl4(RedButtons);
}

int main2(){
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
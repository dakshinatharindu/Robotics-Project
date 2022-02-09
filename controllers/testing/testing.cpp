
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

#define TIME_STEP 32

// All the webots classes are defined in the "webots" namespace
using namespace webots;


int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  Motor *servo = robot->getMotor("servo");
  servo->setPosition(INFINITY);
  servo->setVelocity(0);



  while (robot->step(TIME_STEP) != -1) {
   servo->setVelocity(4);
  };

  

  delete robot;
  return 0;
}

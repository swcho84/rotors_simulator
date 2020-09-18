#include "rotors_control/joystick_trjvel_controller.h"

using namespace std;
using namespace ros;
using namespace Eigen;

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char** argv)
{
  // Set up ROS.
  init(argc, argv, "joy_trjvel_cntl_node");
  NodeHandle nh("");

  JoyTrjVelCntl joyTrajVelCon;
  nh.getParam("/joystick_velocity_controller/name_cntl", joyTrajVelCon.strJoyCntlName);

  // Tell ROS how fast to run this node.
  Rate loopRate(100);

  while (ok())
  {
    joyTrajVelCon.MainLoop();

    spinOnce();
    loopRate.sleep();
  }  

  joyTrajVelCon.~JoyTrjVelCntl();

  return 0;
}  // end main()
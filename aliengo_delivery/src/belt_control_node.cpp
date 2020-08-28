#include <ros/ros.h>
#include "aliengo_delivery/belt_control.hpp"

using namespace std;
using namespace ros;
 
//main node using class
int main(int argc, char** argv) {
  ros::init(argc, argv, "belt_control_node");
  ros::NodeHandle nodeHandle("~");
  aliengo_delivery::BeltControl beltControl(nodeHandle);
  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
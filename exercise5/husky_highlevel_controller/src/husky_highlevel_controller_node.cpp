#include <ros/ros.h>
#include "husky_highlevel_controller/HuskyHighLevelController.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "husky_highlevel_controller");
  ros::NodeHandle nodeHandle("~");

  husky_highlevel_controller::HuskyHighLevelController HuskyHighLevelController(nodeHandle);

  ros::spin();

  return 0;
}

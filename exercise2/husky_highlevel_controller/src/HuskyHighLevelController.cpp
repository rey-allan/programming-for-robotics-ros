#include "husky_highlevel_controller/HuskyHighLevelController.hpp"

namespace husky_highlevel_controller
{
HuskyHighLevelController::HuskyHighLevelController(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle)
{
  nodeHandle_.getParam("scan_topic", scanTopic_);
  nodeHandle_.getParam("topic_queue_size", scanTopicQueueSize_);
  // You must hold on to the subscriber object because whenever it goes out of scope, the subscription is terminated!
  scanSubscriber_ =
      nodeHandle_.subscribe(scanTopic_, scanTopicQueueSize_, &HuskyHighLevelController::scanTopicCallback, this);
  ROS_INFO("Successfully initialized husky_highlevel_controller node");
}

HuskyHighLevelController::~HuskyHighLevelController()
{
}

void HuskyHighLevelController::scanTopicCallback(const sensor_msgs::LaserScan& message)
{
  ROS_INFO("Smallest distance: %f", std::min_element(message.ranges.begin(), message.ranges.end())[0]);
}
}

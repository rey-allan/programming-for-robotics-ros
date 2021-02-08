#include "husky_highlevel_controller/HuskyProximityStopController.hpp"

namespace husky_highlevel_controller
{
HuskyProximityStopController::HuskyProximityStopController(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle)
{
  nodeHandle_.getParam("min_stop_distance", minStopDistance_);
  nodeHandle_.getParam("scan_topic", scanTopic_);
  nodeHandle_.getParam("topic_queue_size", scanTopicQueueSize_);
  nodeHandle_.getParam("start_stop_topic", startStopTopic_);
  nodeHandle_.getParam("start_stop_topic_queue_size", startStopTopicQueueSize_);

  scanSubscriber_ =
      nodeHandle_.subscribe(scanTopic_, scanTopicQueueSize_, &HuskyProximityStopController::scanTopicCallback, this);
  startStopPublisher_ = nodeHandle_.advertise<std_msgs::Bool>(startStopTopic_, startStopTopicQueueSize_);

  ROS_INFO("Successfully initialized husky_proximity_stop_controller node");
}

HuskyProximityStopController::~HuskyProximityStopController()
{
}

void HuskyProximityStopController::scanTopicCallback(const sensor_msgs::LaserScan& message)
{
  // Get the minimum distance to the obstacle
  float distance = std::min_element(message.ranges.begin(), message.ranges.end())[0];
  // Publish the start/stop signal based on whether the distance breaches the threshold
  bool signal = distance < minStopDistance_;
  startStopPublisher_.publish(signal);
}
}  // namespace husky_highlevel_controller

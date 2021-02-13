#include "husky_highlevel_controller/HuskyProximityStopController.hpp"

namespace husky_highlevel_controller
{
HuskyProximityStopController::HuskyProximityStopController(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle)
{
  nodeHandle_.getParam("min_stop_distance", minStopDistance_);
  nodeHandle_.getParam("scan_topic", scanTopic_);
  nodeHandle_.getParam("topic_queue_size", scanTopicQueueSize_);

  scanSubscriber_ =
      nodeHandle_.subscribe(scanTopic_, scanTopicQueueSize_, &HuskyProximityStopController::scanTopicCallback, this);
  // The service name has to be prefixed with the node name because it was advertised using a private (`~`) node handler
  startStopClient_ = nodeHandle_.serviceClient<std_srvs::SetBool>("/husky_highlevel_controller_v3/start_stop_husky");

  ROS_INFO("Successfully initialized husky_proximity_stop_controller node");
}

HuskyProximityStopController::~HuskyProximityStopController()
{
}

void HuskyProximityStopController::scanTopicCallback(const sensor_msgs::LaserScan& message)
{
  // Get the minimum distance to the obstacle
  float distance = std::min_element(message.ranges.begin(), message.ranges.end())[0];
  // Send a request to the start/stop service based on whether the distance breaches the threshold
  bool signal = distance < minStopDistance_;
  startStopRequest_.request.data = signal;
  startStopClient_.call(startStopRequest_);
}
}  // namespace husky_highlevel_controller

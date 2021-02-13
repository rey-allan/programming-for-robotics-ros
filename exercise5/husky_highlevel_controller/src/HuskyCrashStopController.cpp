#include "husky_highlevel_controller/HuskyCrashStopController.hpp"

namespace husky_highlevel_controller
{
HuskyCrashStopController::HuskyCrashStopController(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle)
{
  nodeHandle_.getParam("angular_velocity_y_threshold", angularVelocityYThreshold_);
  nodeHandle_.getParam("imu_topic", imuTopic_);
  nodeHandle_.getParam("imu_topic_queue_size", imuTopicQueueSize_);

  imuSubscriber_ =
      nodeHandle_.subscribe(imuTopic_, imuTopicQueueSize_, &HuskyCrashStopController::imuTopicCallback, this);
  // The service name has to be prefixed with the node name because it was advertised using a private (`~`) node handler
  startStopClient_ = nodeHandle_.serviceClient<std_srvs::SetBool>("/husky_highlevel_controller_v3/start_stop_husky");

  ROS_INFO("Successfully initialized husky_crash_stop_controller node");
}

HuskyCrashStopController::~HuskyCrashStopController()
{
}

void HuskyCrashStopController::imuTopicCallback(const sensor_msgs::Imu& message)
{
  // Send a request to the start/stop service based on whether the velocity breaches the threshold
  bool signal = message.angular_velocity.y < angularVelocityYThreshold_;
  startStopRequest_.request.data = signal;
  startStopClient_.call(startStopRequest_);

  // If we are stopping means the robot crashed so we stop subscribing to the imu data topic as well
  // This is to avoid receiving data that might prompt Husky to start moving again
  if (signal)
  {
    imuSubscriber_.shutdown();
  }
}
}  // namespace husky_highlevel_controller

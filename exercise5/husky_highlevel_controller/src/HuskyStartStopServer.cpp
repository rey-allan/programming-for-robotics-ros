#include "husky_highlevel_controller/HuskyStartStopServer.hpp"

namespace husky_highlevel_controller
{
HuskyStartStopServer::HuskyStartStopServer(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle)
{
  nodeHandle_.getParam("start_stop_topic", topic_);
  nodeHandle_.getParam("start_stop_topic_queue_size", topicQueueSize_);

  service_ = nodeHandle_.advertiseService("start_stop_husky", &HuskyStartStopServer::startOrStop, this);
  publisher_ = nodeHandle_.advertise<std_msgs::Bool>(topic_, topicQueueSize_);

  ROS_INFO("Successfully initialized start_stop_husky server");
}

HuskyStartStopServer::~HuskyStartStopServer()
{
}

bool HuskyStartStopServer::startOrStop(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
  bool signal = request.data;
  ROS_INFO("Received signal: [%d]", signal);

  // Publish the signal to the start/stop topic
  publisher_.publish(signal);
  response.success = true;

  return true;
}
}  // namespace husky_highlevel_controller

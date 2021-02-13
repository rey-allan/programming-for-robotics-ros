#include "husky_highlevel_controller/HuskyHighLevelController.hpp"

namespace husky_highlevel_controller
{
HuskyHighLevelController::HuskyHighLevelController(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle)
{
  nodeHandle_.getParam("scan_topic", scanTopic_);
  nodeHandle_.getParam("topic_queue_size", scanTopicQueueSize_);
  nodeHandle_.getParam("cmd_topic", cmdTopic_);
  nodeHandle_.getParam("cmd_topic_queue_size", cmdTopicQueueSize_);
  nodeHandle_.getParam("rviz_topic", rvizTopic_);
  nodeHandle_.getParam("rviz_topic_queue_size", rvizTopicQueueSize_);
  nodeHandle_.getParam("forward_vel", forwardVel_);
  nodeHandle_.getParam("angular_vel", angularVel_);

  // You must hold on to the subscriber/publisher objects because whenever they go out of scope, subs are terminated!
  scanSubscriber_ =
      nodeHandle_.subscribe(scanTopic_, scanTopicQueueSize_, &HuskyHighLevelController::scanTopicCallback, this);
  cmdPublisher_ = nodeHandle_.advertise<geometry_msgs::Twist>(cmdTopic_, cmdTopicQueueSize_);
  rvizPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>(rvizTopic_, rvizTopicQueueSize_);

  service_ = nodeHandle_.advertiseService("start_stop_husky", &HuskyHighLevelController::startOrStop, this);

  // Initially, Husky will start moving towards the pillar
  stopHusky_ = false;

  ROS_INFO("Successfully initialized husky_highlevel_controller_v3 node");
}

HuskyHighLevelController::~HuskyHighLevelController()
{
}

void HuskyHighLevelController::scanTopicCallback(const sensor_msgs::LaserScan& message)
{
  // Since our robot is alone in the world with a single pillar, we know that the smallest distance recorded
  // must be to some part of that pillar; we can then extract the position of this measurement in the `ranges`
  // The position is actually the number of angle increments that have occurred from the first ray (position 0)
  float distance = std::min_element(message.ranges.begin(), message.ranges.end())[0];
  int increments = std::min_element(message.ranges.begin(), message.ranges.end()) - message.ranges.begin();
  // To compute the angle of this ray, we add to the min angle (the first ray) the number of angle increments
  // See: https://stackoverflow.com/a/61998432
  float angle = message.angle_min + message.angle_increment * increments;

  // With the angle, we can now compute the x,y position of the pillar
  // We know that our robot is always facing forward; therefore, the pillar and the laser of the robot form a
  // right-angled triangle from which the position can be computed using cosine and sine
  // See the exercise instructions for a nice drawing of this, and https://www.mathsisfun.com/sine-cosine-tangent.html
  float x = cos(angle) * distance;
  float y = sin(angle) * distance;

  // Now we can write a simple P (Proportional) controller to drive Husky towards the pillar!
  geometry_msgs::Twist twist;
  // Use the negation of `stopHusky_` as an int to automatically start/stop Husky
  // For example, if we receive a signal to stop Husky, then stopHusky=true which negated becomes false (as int is 0)
  // and when multiplied by it, both the forward and angular velocities will become 0, effectively stopping the robot!
  twist.linear.x = forwardVel_ * (int)(!stopHusky_);
  // We want to rotate with respect to z-axis (the one pointing up) proportional to the angle of the pillar
  // We use `0 - angle` as opposed to `angle - 0` because the frame of the laser is rotated so we need to
  // rotate the robot to the opposite side (see exercise instructions for reference)
  twist.angular.z = (0 - angle) * angularVel_ * (int)(!stopHusky_);
  cmdPublisher_.publish(twist);

  // Finally, publish the extracted point as an RViz marker
  visualization_msgs::Marker marker;
  // We need to publish it based on the laser's frame of reference since the position was computed against it
  marker.header.frame_id = "base_laser";
  marker.header.stamp = ros::Time();
  marker.ns = "husky";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  rvizPublisher_.publish(marker);
}

bool HuskyHighLevelController::startOrStop(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
  stopHusky_ = request.data;
  response.success = true;

  return true;
}
}  // namespace husky_highlevel_controller

#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

namespace husky_highlevel_controller
{
/*
 * Implements the logic behind the node interfacing with ROS.
 */
class HuskyHighLevelController
{
public:
  HuskyHighLevelController(ros::NodeHandle& nodeHandle);

  virtual ~HuskyHighLevelController();

private:
  /**
   * Processes messages from the `/scan` topic.
   */
  void scanTopicCallback(const sensor_msgs::LaserScan& message);

  ros::NodeHandle& nodeHandle_;
  ros::Subscriber scanSubscriber_;
  ros::Publisher cmdPublisher_;
  ros::Publisher rvizPublisher_;
  std::string scanTopic_;
  int scanTopicQueueSize_;
  std::string cmdTopic_;
  int cmdTopicQueueSize_;
  std::string rvizTopic_;
  int rvizTopicQueueSize_;
  float forwardVel_;
  float angularVel_;
};
}

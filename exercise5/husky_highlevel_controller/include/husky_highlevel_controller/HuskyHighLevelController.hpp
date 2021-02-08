#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>

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

  /**
   * Processes message from the `/start_stop_husky` topic.
   */
  void startStopCallback(const std_msgs::Bool& message);

  ros::NodeHandle& nodeHandle_;
  ros::Subscriber scanSubscriber_;
  ros::Subscriber startStopSubscriber_;
  ros::Publisher cmdPublisher_;
  ros::Publisher rvizPublisher_;
  std::string scanTopic_;
  int scanTopicQueueSize_;
  std::string cmdTopic_;
  int cmdTopicQueueSize_;
  std::string rvizTopic_;
  int rvizTopicQueueSize_;
  std::string startStopTopic_;
  int startStopTopicQueueSize_;
  float forwardVel_;
  float angularVel_;
  bool stopHusky_;
};
}  // namespace husky_highlevel_controller

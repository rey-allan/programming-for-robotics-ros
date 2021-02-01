#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

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
  std::string scanTopic_;
  int scanTopicQueueSize_;
};
}

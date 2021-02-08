#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>

namespace husky_highlevel_controller
{
/*
 * Stops Husky if it is too close to an obstacle using the laser measurements.
 */
class HuskyProximityStopController
{
public:
  HuskyProximityStopController(ros::NodeHandle& nodeHandle);

  virtual ~HuskyProximityStopController();

private:
  /**
   * Processes messages from the `/scan` topic.
   */
  void scanTopicCallback(const sensor_msgs::LaserScan& message);

  ros::NodeHandle& nodeHandle_;
  ros::Subscriber scanSubscriber_;
  ros::Publisher startStopPublisher_;
  std::string scanTopic_;
  int scanTopicQueueSize_;
  std::string startStopTopic_;
  int startStopTopicQueueSize_;
  float minStopDistance_;
};
}  // namespace husky_highlevel_controller

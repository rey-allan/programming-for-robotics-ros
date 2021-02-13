#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/SetBool.h>

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
  ros::ServiceClient startStopClient_;
  std_srvs::SetBool startStopRequest_;
  std::string scanTopic_;
  int scanTopicQueueSize_;
  float minStopDistance_;
};
}  // namespace husky_highlevel_controller

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/SetBool.h>

namespace husky_highlevel_controller
{
/*
 * Stops Husky if it detects a crash using IMU data.
 */
class HuskyCrashStopController
{
public:
  HuskyCrashStopController(ros::NodeHandle& nodeHandle);

  virtual ~HuskyCrashStopController();

private:
  /**
   * Processes messages from the `/imu_data` topic.
   */
  void imuTopicCallback(const sensor_msgs::Imu& message);

  ros::NodeHandle& nodeHandle_;
  ros::Subscriber imuSubscriber_;
  ros::ServiceClient startStopClient_;
  std_srvs::SetBool startStopRequest_;
  std::string imuTopic_;
  int imuTopicQueueSize_;
  float angularVelocityYThreshold_;
};
}  // namespace husky_highlevel_controller

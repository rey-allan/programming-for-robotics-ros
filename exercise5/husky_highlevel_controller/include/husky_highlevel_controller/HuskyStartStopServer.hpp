#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>

namespace husky_highlevel_controller
{
/*
 * Implements a service server for starting/stopping Husky.
 */
class HuskyStartStopServer
{
public:
  HuskyStartStopServer(ros::NodeHandle& nodeHandle);

  virtual ~HuskyStartStopServer();

private:
  /**
   * Callback that sends a signal for starting/stopping Husky.
   */
  bool startOrStop(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);

  ros::NodeHandle& nodeHandle_;
  ros::ServiceServer service_;
  ros::Publisher publisher_;
  std::string topic_;
  int topicQueueSize_;
};
}  // namespace husky_highlevel_controller

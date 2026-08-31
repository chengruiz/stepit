#ifndef STEPIT_ROS_CONTROL_H_
#define STEPIT_ROS_CONTROL_H_

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <stepit_ros_msgs/Control.h>
#include <stepit/control_input.h>

namespace stepit {
class RosMsgControl : public ControlInput {
 public:
  RosMsgControl();
  ~RosMsgControl() override = default;
  bool available() const override { return received_; }
  void poll() override {}

 private:
  void msgCallback(const std_msgs::String::ConstPtr &msg);

  ros::Subscriber sub_;
  std::atomic<bool> received_{false};
};

class RosSrvControl : public ControlInput {
 public:
  RosSrvControl();
  ~RosSrvControl() override = default;
  bool available() const override { return received_; }
  void poll() override {}

 private:
  bool srvCallback(stepit_ros_msgs::Control::Request &req, stepit_ros_msgs::Control::Response &res);

  ros::ServiceServer srv_;
  std::atomic<bool> received_{false};
};
}  // namespace stepit

#endif  // STEPIT_ROS_CONTROL_H_
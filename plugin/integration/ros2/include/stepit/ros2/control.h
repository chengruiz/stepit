#ifndef STEPIT_ROS2_CONTROL_H_
#define STEPIT_ROS2_CONTROL_H_

#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <stepit_ros2_msgs/srv/control.hpp>
#include <stepit/control_input.h>

namespace stepit {
class Ros2MsgControl final : public ControlInput {
 public:
  Ros2MsgControl();
  ~Ros2MsgControl() override = default;
  [[nodiscard]] bool available() const override { return received_; }
  void poll() override {}

 private:
  void msgCallback(std_msgs::msg::String::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::atomic<bool> received_{false};
};

class Ros2SrvControl final : public ControlInput {
 public:
  Ros2SrvControl();
  ~Ros2SrvControl() override = default;
  [[nodiscard]] bool available() const override { return received_; }
  void poll() override {}

 private:
  using ControlSrv = stepit_ros2_msgs::srv::Control;
  void srvCallback(ControlSrv::Request::SharedPtr req, ControlSrv::Response::SharedPtr res);

  rclcpp::Service<ControlSrv>::SharedPtr srv_;
  std::atomic<bool> received_{false};
};
}  // namespace stepit

#endif  // STEPIT_ROS2_CONTROL_H_

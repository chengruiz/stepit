#ifndef STEPIT_ROS2_JOYSTICK_H_
#define STEPIT_ROS2_JOYSTICK_H_

#include <atomic>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <unitree_go/msg/wireless_controller.hpp>

#include <stepit/joystick/joystick.h>

namespace u2ros2_msg = unitree_go::msg;

namespace stepit {
namespace joystick {
class Unitree2Ros2Joystick final : public Joystick {
 public:
  Unitree2Ros2Joystick();
  bool connected() const override;
  void getState(State &state) override;

 private:
  void callback(const u2ros2_msg::WirelessController::SharedPtr msg);

  rclcpp::Subscription<u2ros2_msg::WirelessController>::SharedPtr joy_sub_;
  std::mutex mutex_;
  State state_;
  u2ros2_msg::WirelessController msg_;
  std::atomic<std::size_t> tick_{0};
};
}  // namespace joystick
}  // namespace stepit

#endif  // STEPIT_ROS2_JOYSTICK_H_

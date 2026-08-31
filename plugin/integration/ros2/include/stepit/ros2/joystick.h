#ifndef STEPIT_ROS2_JOYSTICK_H_
#define STEPIT_ROS2_JOYSTICK_H_

#include <atomic>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <stepit/joystick/joystick.h>
#include <stepit/joystick/keymap.h>

namespace stepit::joystick {
class Ros2Joystick final : public Joystick {
 public:
  Ros2Joystick();
  explicit Ros2Joystick(const Keymap &keymap);
  [[nodiscard]] bool connected() const override;
  void getState(State &state) override;

 private:
  void callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void axisHandler(std::size_t aid, float value);
  void buttonHandler(std::size_t bid, bool value);

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  std::mutex mutex_;
  std::atomic<bool> connected_{false};
  sensor_msgs::msg::Joy msg_;
  rclcpp::Time stamp_{0, 0, RCL_ROS_TIME};

  Slots slots_;
  Keymap keymap_;
};
}  // namespace stepit::joystick

#endif  // STEPIT_ROS2_JOYSTICK_H_

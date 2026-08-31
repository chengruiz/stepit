#ifndef STEPIT_ROS_JOYSTICK_H_
#define STEPIT_ROS_JOYSTICK_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <stepit/joystick/joystick.h>
#include <stepit/joystick/keymap.h>

namespace stepit {
namespace joystick {
class RosJoystick final : public Joystick {
 public:
  RosJoystick();
  explicit RosJoystick(const Keymap &keymap);
  bool connected() const override;
  void getState(State &state) override;

 private:
  void callback(const sensor_msgs::Joy::ConstPtr &msg);
  void axisHandler(std::size_t aid, float value);
  void buttonHandler(std::size_t bid, bool value);

  ros::Subscriber joy_sub_;
  std::mutex mutex_;

  std::atomic<bool> connected_{false};
  sensor_msgs::Joy msg_;
  ros::Time stamp_;

  Slots slots_;
  Keymap keymap_;
};
}  // namespace joystick
}  // namespace stepit

#endif  // STEPIT_ROS_JOYSTICK_H_

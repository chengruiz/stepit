#include <stepit/robot/unitree2/joystick.h>
#include <stepit/robot/unitree2_ros2/joystick.h>
#include <stepit/ros2/node.h>

namespace stepit {
namespace joystick {
Unitree2Ros2Joystick::Unitree2Ros2Joystick() {
  joy_sub_ = getNode()->create_subscription<u2ros2_msg::WirelessController>(
      "/wirelesscontroller", 1, [this](const u2ros2_msg::WirelessController::SharedPtr msg) { callback(msg); });
}

bool Unitree2Ros2Joystick::connected() const { return tick_ != 0; }

void Unitree2Ros2Joystick::getState(State &state) {
  std::lock_guard<std::mutex> _(mutex_);
  state = state_;
  for (auto &button : state_.buttons()) button.resetTransientStates();
}

void Unitree2Ros2Joystick::callback(const u2ros2_msg::WirelessController::SharedPtr msg) {
  std::lock_guard<std::mutex> _(mutex_);
  Unitree2Joystick::updateState(state_, msg->lx, msg->ly, msg->rx, msg->ry, msg->keys);
  tick_ += 1;
}

STEPIT_REGISTER_JOYSTICK(unitree2_ros2, kDefPriority, Joystick::make<Unitree2Ros2Joystick>);
}  // namespace joystick
}  // namespace stepit

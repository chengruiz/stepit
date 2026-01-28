#include <stepit/utils.h>
#include <stepit/ros2/joystick.h>
#include <stepit/ros2/node.h>

namespace stepit::joystick {
std::string getJoyName() {
  std::string joy_name;
  if (not getenv("STEPIT_JOY_NAME", joy_name)) {
    STEPIT_ASSERT(getNode()->get_parameter("joy_name", joy_name), "Parameter 'joy_name' is not defined.");
  }
  return joy_name;
}

Ros2Joystick::Ros2Joystick()
    : Ros2Joystick(loadConfigFile(fmt::format("joystick/{}.yml", getJoyName()))) {}

Ros2Joystick::Ros2Joystick(const Keymap &keymap) : keymap_(keymap) {
  std::string topic_name = "/joy";
  getNode()->get_parameter_or("joy_topic", topic_name, std::string{"/joy"});
  joy_sub_ = getNode()->create_subscription<sensor_msgs::msg::Joy>(
      topic_name, getDefaultQoS(), [this](const sensor_msgs::msg::Joy::SharedPtr msg) { callback(msg); });
}

bool Ros2Joystick::connected() const { return connected_ and getElapsedTime(stamp_) < 0.1; }

void Ros2Joystick::getState(State &state) {
  if (not connected_) {
    state = {};
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  populateStateFromSlots(keymap_, slots_, state);
  for (auto &button : slots_.buttons) button.resetTransientStates();
}

void Ros2Joystick::callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  stamp_ = msg->header.stamp;
  if (not connected_) {
    for (std::size_t i{}; i < msg->axes.size(); ++i) {
      axisHandler(i, msg->axes[i]);
    }
    for (std::size_t i{}; i < msg->buttons.size(); ++i) {
      buttonHandler(i, msg->buttons[i]);
    }
    connected_ = true;
  } else {
    for (std::size_t i{}; i < msg->axes.size(); ++i) {
      if (msg->axes[i] != msg_.axes[i]) {
        axisHandler(i, msg->axes[i]);
      }
    }
    for (std::size_t i{}; i < msg->buttons.size(); ++i) {
      if (msg->buttons[i] != msg_.buttons[i]) {
        buttonHandler(i, msg->buttons[i]);
      }
    }
  }
  msg_ = *msg;
}

void Ros2Joystick::axisHandler(std::size_t aid, float value) {
  if (aid >= Slots::NAxes) return;
  value            = -value;
  slots_.axes[aid] = value;
  slots_.buttons[Slots::NButtons + 2 * aid].update(value < -0.99f);
  slots_.buttons[Slots::NButtons + 2 * aid + 1].update(value > 0.99f);
}

void Ros2Joystick::buttonHandler(std::size_t bid, bool value) {
  if (bid >= Slots::NButtons) return;
  slots_.buttons[bid].update(value);
}

STEPIT_REGISTER_JOYSTICK(ros2, kDefPriority, Joystick::make<Ros2Joystick>);
STEPIT_REGISTER_JOYSTICK(ros2_xbox, kDefPriority, [] { return std::make_unique<Ros2Joystick>(getXboxKeymap()); });
}  // namespace stepit::joystick

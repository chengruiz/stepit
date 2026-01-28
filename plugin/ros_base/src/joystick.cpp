#include <stepit/utils.h>
#include <stepit/ros/joystick.h>
#include <stepit/ros/node_handle.h>

namespace stepit {
namespace joystick {
std::string getJoyName() {
  std::string joy_name;
  if (not getenv("STEPIT_JOY_NAME", joy_name)) {
    STEPIT_ASSERT(getNodeHandle().getParam("joy_name", joy_name), "Parameter 'joy_name' is not defined.");
  }
  return joy_name;
}

RosJoystick::RosJoystick()
    : RosJoystick(loadConfigFile(fmt::format("joystick/{}.yml", getJoyName()))) {}

RosJoystick::RosJoystick(const Keymap &keymap) : keymap_(keymap) {
  std::string topic_name{"/joy"};
  getNodeHandle().getParam("joy_topic", topic_name);
  joy_sub_ = getNodeHandle().subscribe(topic_name, 10, &RosJoystick::callback, this,
                                       ros::TransportHints().tcpNoDelay());
}

bool RosJoystick::connected() const { return connected_ and getElapsedTime(stamp_) < 0.1; }

void RosJoystick::getState(State &state) {
  if (not connected_) {
    state = {};
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  populateStateFromSlots(keymap_, slots_, state);

  for (auto &button : slots_.buttons) button.resetTransientStates();
}

void RosJoystick::callback(const sensor_msgs::Joy::ConstPtr &msg) {
  std::lock_guard<std::mutex> _(mutex_);
  stamp_ = msg->header.stamp;
  if (not connected_) {
    for (int i{}; i < msg->axes.size(); ++i) {
      axisHandler(i, msg->axes[i]);
    }
    for (int i{}; i < msg->buttons.size(); ++i) {
      buttonHandler(i, msg->buttons[i]);
    }
    connected_ = true;
  } else {
    for (int i{}; i < msg->axes.size(); ++i) {
      if (msg->axes[i] != msg_.axes[i]) {
        axisHandler(i, msg->axes[i]);
      }
    }
    for (int i{}; i < msg->buttons.size(); ++i) {
      if (msg->buttons[i] != msg_.buttons[i]) {
        buttonHandler(i, msg->buttons[i]);
      }
    }
  }
  msg_ = *msg;
}

void RosJoystick::axisHandler(std::size_t aid, float value) {
  if (aid >= Slots::NAxes) return;
  value = -value;

  slots_.axes[aid] = value;
  // Convert the analogue axis to two virtual buttons, one for each direction
  slots_.buttons[Slots::NButtons + 2 * aid].update(value < -0.99);
  slots_.buttons[Slots::NButtons + 2 * aid + 1].update(value > 0.99);
}

void RosJoystick::buttonHandler(std::size_t bid, bool value) {
  if (bid >= Slots::NButtons) return;
  slots_.buttons[bid].update(value);
}

STEPIT_REGISTER_JOYSTICK(ros, kDefPriority, Joystick::make<RosJoystick>);
STEPIT_REGISTER_JOYSTICK(ros_xbox, kDefPriority, [] { return std::make_unique<RosJoystick>(getXboxKeymap()); });
}  // namespace joystick
}  // namespace stepit

#include <stepit/robot/unitree2/joystick.h>

namespace stepit {
namespace joystick {
Unitree2Joystick::Unitree2Joystick() {
  Unitree2ServiceClient::initialize();
  sub_ = std::make_shared<u2_sdk::ChannelSubscriber<u2_msg::WirelessController_>>("rt/wirelesscontroller");
  sub_->InitChannel([this](const void *msg) { callback(static_cast<const u2_msg::WirelessController_ *>(msg)); }, 1);
}

bool Unitree2Joystick::connected() const { return tick_ != 0; }

void Unitree2Joystick::getState(State &state) {
  std::lock_guard<std::mutex> _(mutex_);
  state = state_;
  for (auto &button : state_.buttons()) button.resetTransientStates();
}

void Unitree2Joystick::updateState(State &state, float lx, float ly, float rx, float ry, uint16_t keys_data) {
  Keys keys{.value = keys_data};

  state.A().update(keys.A);
  state.B().update(keys.B);
  state.X().update(keys.X);
  state.Y().update(keys.Y);
  state.LB().update(keys.L1);
  state.RB().update(keys.R1);
  state.Select().update(keys.select);
  state.Start().update(keys.start);
  state.LAS().update(keys.L1 and keys.L2);
  state.RAS().update(keys.R1 and keys.R2);
  state.Up().update(keys.up);
  state.Down().update(keys.down);
  state.Left().update(keys.left);
  state.Right().update(keys.right);

  state.las_x() = lx;
  state.las_y() = -ly;
  state.ras_x() = rx;
  state.ras_y() = -ry;
  state.lt()    = static_cast<float>(keys.L2) * 2 - 1;
  state.rt()    = static_cast<float>(keys.R2) * 2 - 1;
}

void Unitree2Joystick::callback(const u2_msg::WirelessController_ *msg) {
  std::lock_guard<std::mutex> _(mutex_);
  updateState(state_, msg->lx(), msg->ly(), msg->rx(), msg->ry(), msg->keys());
  tick_ += 1;
}

STEPIT_REGISTER_JOYSTICK(unitree2, kDefPriority, Joystick::make<Unitree2Joystick>);
STEPIT_REGISTER_CTRLINPUT(joystick_unitree2, kDefPriority,
                          []() { return std::make_unique<JoystickControl>("unitree2"); });
}  // namespace joystick
}  // namespace stepit

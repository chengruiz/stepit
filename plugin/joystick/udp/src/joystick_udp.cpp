#include <udp_gamepad/retroid_gamepad.h>
#include <udp_gamepad/skydroid_gamepad.h>

#include <stepit/joystick/joystick.h>

namespace stepit {
namespace joystick {
template <typename Native>
class UdpJoystick : public Joystick {
 public:
  UdpJoystick() {
    native_.SetUpdateCallback([this](const auto &keys, uint32_t tick) { callback(keys, tick); });
    native_.StartDataThread();
  }

  bool connected() const override {
    std::lock_guard<std::mutex> _(mutex_);
    return tick_ != 0 and getElapsedTime<MSec>(stamp_) < timeout_;
  }

  void getState(State &state) override {
    std::lock_guard<std::mutex> _(mutex_);
    state = state_;
    for (auto &button : state_.buttons()) button.resetTransientStates();
  }

 protected:
  using NativeType = Native;
  using KeysType   = typename Native::Keys;

  virtual void callback(const KeysType &keys, uint32_t tick) = 0;

  Native native_;
  State state_;
  uint32_t tick_{0}, timeout_{1500};
  TimePoint stamp_;
  mutable std::mutex mutex_;
};

class RetroidJoystick final : public UdpJoystick<udp_gamepad::RetroidGamepad> {
  void callback(const KeysType &keys, uint32_t tick) override {
    std::lock_guard<std::mutex> _(mutex_);
    tick_  = tick;
    stamp_ = SteadyClock::now();

    state_.A().update(keys.A);
    state_.B().update(keys.B);
    state_.X().update(keys.X);
    state_.Y().update(keys.Y);
    state_.LB().update(keys.L1);
    state_.RB().update(keys.R1);
    state_.Select().update(keys.select);
    state_.Start().update(keys.start);
    state_.LAS().update(keys.left_axis_button);
    state_.RAS().update(keys.right_axis_button);
    state_.Up().update(keys.up);
    state_.Down().update(keys.down);
    state_.Left().update(keys.left);
    state_.Right().update(keys.right);

    state_.las_x() = keys.left_axis_x;
    state_.las_y() = -keys.left_axis_y;
    state_.ras_x() = keys.right_axis_x;
    state_.ras_y() = -keys.right_axis_y;
    state_.lt()    = static_cast<float>(keys.L2) * 2 - 1;
    state_.rt()    = static_cast<float>(keys.R2) * 2 - 1;
  }
};

STEPIT_REGISTER_JOYSTICK(retroid, kDefPriority, Joystick::make<RetroidJoystick>);
}  // namespace joystick
}  // namespace stepit

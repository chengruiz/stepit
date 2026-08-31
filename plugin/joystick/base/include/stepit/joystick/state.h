#ifndef STEPIT_JOYSTICK_STATE_H_
#define STEPIT_JOYSTICK_STATE_H_

#include <array>
#include <ostream>

#include <stepit/utils.h>

namespace stepit {
namespace joystick {
using Axis = float;

struct Button {
  bool on_press{false}, pressed{false}, on_release{false};

  void update(bool pressed) {
    if (pressed and not this->pressed) {
      this->on_press = true;
    } else if (this->pressed and not pressed) {
      this->on_release = true;
    }
    this->pressed = pressed;
  }

  void resetTransientStates() { on_press = on_release = false; }
};

class State {
  static constexpr std::size_t NButtons = 14;
  static constexpr std::size_t NAxes    = 6;

 public:
  State() { lt() = rt() = -1.; }

  Button &A() { return buttons_[0]; }
  Button &B() { return buttons_[1]; }
  Button &X() { return buttons_[2]; }
  Button &Y() { return buttons_[3]; }
  Button &LB() { return buttons_[4]; }
  Button &RB() { return buttons_[5]; }
  Button &Select() { return buttons_[6]; }
  Button &Start() { return buttons_[7]; }
  Button &LAS() { return buttons_[8]; }
  Button &RAS() { return buttons_[9]; }
  Button &Up() { return buttons_[10]; }
  Button &Down() { return buttons_[11]; }
  Button &Left() { return buttons_[12]; }
  Button &Right() { return buttons_[13]; }
  std::array<Button, NButtons> &buttons() { return buttons_; }

  const Button &A() const { return buttons_[0]; }
  const Button &B() const { return buttons_[1]; }
  const Button &X() const { return buttons_[2]; }
  const Button &Y() const { return buttons_[3]; }
  const Button &LB() const { return buttons_[4]; }
  const Button &RB() const { return buttons_[5]; }
  const Button &Select() const { return buttons_[6]; }
  const Button &Start() const { return buttons_[7]; }
  const Button &LAS() const { return buttons_[8]; }
  const Button &RAS() const { return buttons_[9]; }
  const Button &Up() const { return buttons_[10]; }
  const Button &Down() const { return buttons_[11]; }
  const Button &Left() const { return buttons_[12]; }
  const Button &Right() const { return buttons_[13]; }
  const std::array<Button, NButtons> &buttons() const { return buttons_; }

  Axis &las_x() { return axes_[0]; }
  Axis &las_y() { return axes_[1]; }
  Axis &ras_x() { return axes_[2]; }
  Axis &ras_y() { return axes_[3]; }
  Axis &lt() { return axes_[4]; }
  Axis &rt() { return axes_[5]; }
  std::array<Axis, NAxes> &axes() { return axes_; }

  Axis las_x() const { return axes_[0]; }
  Axis las_y() const { return axes_[1]; }
  Axis ras_x() const { return axes_[2]; }
  Axis ras_y() const { return axes_[3]; }
  Axis lt() const { return axes_[4]; }
  Axis rt() const { return axes_[5]; }
  const std::array<Axis, NAxes> &axes() const { return axes_; }

 private:
  std::array<Button, 14> buttons_{};
  std::array<Axis, 6> axes_{};  // ranging from [-1, 1]
};

inline std::ostream &operator<<(std::ostream &os, const Button &button) {
  os << (button.pressed ? '1' : '0');
  if (button.on_press) os << '+';
  if (button.on_release) os << '-';
  if (not button.on_press and not button.on_release) os << ' ';
  return os;
}

inline std::ostream &operator<<(std::ostream &os, const State &state) {
  return os << fmt::format(
             "axes[las=({:.2f}, {:.2f}) ras=({:.2f}, {:.2f}) lt={:.2f} rt={:.2f}] "
             "buttons[A={} B={} X={} Y={} LB={} RB={} Select={} Start={} LAS={} RAS={} Up={} Down={} Left={} "
             "Right={}] ",
             state.las_x(), state.las_y(), state.ras_x(), state.ras_y(), state.lt(), state.rt(), state.A(), state.B(),
             state.X(), state.Y(), state.LB(), state.RB(), state.Select(), state.Start(), state.LAS(), state.RAS(),
             state.Up(), state.Down(), state.Left(), state.Right());
}
}  // namespace joystick
}  // namespace stepit

#if FMT_VERSION >= 90000
template <>
struct fmt::formatter<stepit::joystick::Button> : fmt::ostream_formatter {};
template <>
struct fmt::formatter<stepit::joystick::State> : fmt::ostream_formatter {};
#endif
#endif  // STEPIT_JOYSTICK_STATE_H_

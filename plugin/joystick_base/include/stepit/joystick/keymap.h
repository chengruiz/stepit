#ifndef STEPIT_JOYSTICK_KEYMAP_H_
#define STEPIT_JOYSTICK_KEYMAP_H_

#include <stepit/joystick/joystick.h>

namespace stepit {
namespace joystick {
struct Slots {
  static constexpr std::size_t NAxes    = 8;
  static constexpr std::size_t NButtons = 16;

  Axis axes[NAxes]{};
  Button buttons[NButtons + 2 * NAxes]{};
};

struct Keymap {
  Keymap() = default;
  Keymap(const YAML::Node &config);

  std::size_t A      = 0;
  std::size_t B      = 1;
  std::size_t X      = 2;
  std::size_t Y      = 3;
  std::size_t LB     = 4;
  std::size_t RB     = 5;
  std::size_t Select = 6;
  std::size_t Start  = 7;
  std::size_t LAS    = 9;
  std::size_t RAS    = 10;
  std::size_t Up     = Slots::NButtons + 14;
  std::size_t Down   = Slots::NButtons + 15;
  std::size_t Left   = Slots::NButtons + 12;
  std::size_t Right  = Slots::NButtons + 13;

  std::size_t las_x = 0;
  std::size_t las_y = 1;
  std::size_t ras_x = 3;
  std::size_t ras_y = 4;
  std::size_t lt    = 2;
  std::size_t rt    = 5;
};

inline void populateStateFromSlots(const Keymap &kep_map, const Slots &slots, State &state) {
  state.A()      = slots.buttons[kep_map.A];
  state.B()      = slots.buttons[kep_map.B];
  state.X()      = slots.buttons[kep_map.X];
  state.Y()      = slots.buttons[kep_map.Y];
  state.LB()     = slots.buttons[kep_map.LB];
  state.RB()     = slots.buttons[kep_map.RB];
  state.Select() = slots.buttons[kep_map.Select];
  state.Start()  = slots.buttons[kep_map.Start];
  state.LAS()    = slots.buttons[kep_map.LAS];
  state.RAS()    = slots.buttons[kep_map.RAS];
  state.Up()     = slots.buttons[kep_map.Up];
  state.Down()   = slots.buttons[kep_map.Down];
  state.Left()   = slots.buttons[kep_map.Left];
  state.Right()  = slots.buttons[kep_map.Right];

  state.las_x() = slots.axes[kep_map.las_x];
  state.las_y() = slots.axes[kep_map.las_y];
  state.ras_x() = slots.axes[kep_map.ras_x];
  state.ras_y() = slots.axes[kep_map.ras_y];
  state.lt()    = slots.axes[kep_map.lt];
  state.rt()    = slots.axes[kep_map.rt];
}

inline Keymap getXboxKeymap() { return {}; }
}  // namespace joystick
}  // namespace stepit

#endif  // STEPIT_JOYSTICK_KEYMAP_H_

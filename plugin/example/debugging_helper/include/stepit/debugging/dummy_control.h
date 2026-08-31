#ifndef STEPIT_DEBUGGING_DUMMY_CONTROL_H_
#define STEPIT_DEBUGGING_DUMMY_CONTROL_H_

#include <stepit/control_input.h>

namespace stepit {
class DummyControl final : public ControlInput {
 public:
  DummyControl() = default;
  bool available() const override { return true; }
  void poll() override {}
};
}  // namespace stepit

#endif  // STEPIT_DEBUGGING_DUMMY_CONTROL_H_

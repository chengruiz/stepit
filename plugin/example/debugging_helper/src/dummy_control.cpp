#include <stepit/debugging/dummy_control.h>

namespace stepit {
STEPIT_REGISTER_CTRLINPUT(dummy, kMinPriority, ControlInput::make<DummyControl>);
}  // namespace stepit

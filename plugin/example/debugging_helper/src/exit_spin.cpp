#include <stepit/debugging/exit_spin.h>

namespace stepit {
int ExitSpin::spin() {
  // Exit immediately with status 0
  return 0;
}

STEPIT_REGISTER_SPIN(exit, kMinPriority, Spin::make<ExitSpin>);
}  // namespace stepit

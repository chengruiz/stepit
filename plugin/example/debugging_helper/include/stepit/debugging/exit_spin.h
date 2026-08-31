#ifndef STEPIT_DEBUGGING_HELPER_EXIT_SPIN_H_
#define STEPIT_DEBUGGING_HELPER_EXIT_SPIN_H_

#include <stepit/spin.h>

namespace stepit {
class ExitSpin final : public Spin {
 public:
  int spin() override;
};
}  // namespace stepit

#endif  // STEPIT_DEBUGGING_HELPER_EXIT_SPIN_H_

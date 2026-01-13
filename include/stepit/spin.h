#ifndef STEPIT_SPIN_H_
#define STEPIT_SPIN_H_

#include <unistd.h>
#include <csignal>
#include <memory>

#include <stepit/registry.h>

namespace stepit {
/**
 * @brief Abstract base interface for spin operations.
 *
 * Spin implementations define a main loop or wait mechanism via the spin() method.
 */
class Spin {
 public:
  using Ptr = std::unique_ptr<Spin>;
  using Reg = RegistrySingleton<Spin>;

  virtual ~Spin() = default;
  /**
   * @brief Run the spin loop or waiting mechanism.
   * @return Exit status (0 for normal exit, non-zero for errors).
   */
  virtual int spin() = 0;
};

using SpinPtr = Spin::Ptr;
using SpinReg = Spin::Reg;

class WaitForSigInt : public Spin {
 public:
  WaitForSigInt() { std::signal(SIGINT, &WaitForSigInt::signalHandler); }

  int spin() override {
    while (not sigint_received_) pause();
    return 0;
  }

 protected:
  static void signalHandler(int) { sigint_received_ = 1; }
  static volatile std::sig_atomic_t sigint_received_;
};

int spin();
}  // namespace stepit

#define STEPIT_REGISTER_SPIN(name, priority, factory) \
  static ::stepit::SpinReg::Registration _spin_##name##_registration(#name, priority, factory)

#endif  // STEPIT_SPIN_H_

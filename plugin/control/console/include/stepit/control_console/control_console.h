#ifndef STEPIT_CONTROL_CONSOLE_H_
#define STEPIT_CONTROL_CONSOLE_H_

#include <atomic>
#include <thread>

#include <stepit/control_input.h>

namespace stepit {
class ConsoleControl : public ControlInput {
 public:
  ConsoleControl();
  ~ConsoleControl() override;
  bool available() const override { return status_; }
  void poll() override {}

 private:
  void readerThread();
  std::atomic<bool> status_{false};
  std::thread reader_thread_;
};
}  // namespace stepit

#endif  // STEPIT_CONTROL_CONSOLE_H_

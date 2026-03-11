#ifndef STEPIT_COMMUNICATION_H_
#define STEPIT_COMMUNICATION_H_

#include <pthread.h>
#include <sys/types.h>

#include <atomic>
#include <mutex>
#include <thread>

#include <stepit/publisher.h>

namespace stepit {
int setThreadCPU(pid_t pid, long cpuid);
int setThreadRT(pthread_t thread);

class Communication final {
 public:
  explicit Communication(const std::string &robot_factory);
  ~Communication();

  const RobotApi::Ptr &api() const { return api_; }
  const RobotSpec &spec() const { return api_->getSpec(); }
  std::size_t dof() const { return dof_; }
  bool isCommunicating() const { return communicating_; }
  bool isConnected() const { return connected_; }
  bool isFrozen() const { return frozen_; }
  bool isActive() const { return active_; }
  std::size_t getFreq() const { return freq_; }
  std::size_t getTick() const { return tick_; }
  LowState getLowState();
  void setLowCmd(const LowCmd &low_cmd);
  void waitUntil(std::size_t tick) const;
  void setActive(bool enabled = true) { active_ = enabled; }
  void setFrozen(bool enabled = true) { frozen_ = enabled; }

  void launch();
  void shutdown();

 private:
  void mainLoop();
  void mainEvent();

  RobotApi::Ptr api_;
  std::thread main_loop_thread_;
  std::atomic<pid_t> thread_id_{-1};

  std::atomic<bool> communicating_{false};
  std::atomic<bool> connected_{false};
  std::atomic<bool> active_{false};
  std::atomic<bool> frozen_{false};

  std::size_t dof_;
  std::size_t freq_;
  std::atomic<std::size_t> tick_{0};
  std::mutex mutex_;
  LowState low_state_;
  LowCmd low_cmd_;
};
}  // namespace stepit

#endif  // STEPIT_COMMUNICATION_H_

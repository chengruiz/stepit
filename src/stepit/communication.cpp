#include <cerrno>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <cmath>

#include <stepit/communication.h>
#include <stepit/logging.h>

namespace stepit {
/**
 * @brief Sets the CPU affinity for a specific thread or process.
 *
 * This function attempts to pin the execution of the thread or process identified by `pid`
 * to the specific CPU core identified by `cpuid`.
 *
 * @param pid The process ID (or thread ID) to set affinity for. If 0, the calling thread is used.
 * @param cpuid The logical ID of the CPU core to bind the thread to.
 *
 * @return The CPU ID actually set if successful, or -1 if the operation failed or the
 *         requested CPU ID was invalid.
 */
int setThreadCPU(pid_t pid, long cpuid) {
  long num_cpus = sysconf(_SC_NPROCESSORS_CONF);
  if (num_cpus <= 0) {
    STEPIT_WARN("Failed to query the number of CPUs.");
    return -1;
  }
  if (cpuid < 0 or cpuid >= num_cpus or cpuid >= static_cast<long>(CPU_SETSIZE)) {
    STEPIT_WARN("Invalid CPU ID '{}'.", cpuid);
    return -1;
  }

  cpu_set_t mask;
  CPU_ZERO(&mask);
  CPU_SET(static_cast<int>(cpuid), &mask);
  int status = sched_setaffinity(pid, sizeof(mask), &mask);
  if (status) {
    int err = errno;
    STEPIT_WARN("Failed to set thread affinity (errno: {}).", err);
    return -1;
  }
  if (sched_getaffinity(pid, sizeof(mask), &mask) != 0) {
    int err = errno;
    STEPIT_WARN("Failed to read thread affinity (errno: {}).", err);
    return -1;
  }
  for (int i{}; i < num_cpus; ++i) {
    if (CPU_ISSET(i, &mask)) return i;
  }
  return -1;
}

/**
 * @brief Sets the scheduling policy of a specific thread to real-time with maximum priority.
 *
 * This function attempts to change the scheduling policy of the provided thread to SCHED_RR
 * and sets its priority to the maximum value allowed for that policy.
 *
 * @param thread The POSIX thread identifier (pthread_t) of the thread to configure.
 * @return int The new scheduling priority of the thread on success, or -1 if setting the parameters failed.
 */
int setThreadRT(pthread_t thread) {
  int policy = SCHED_RR;
  sched_param param{sched_get_priority_max(policy)};
  int status = pthread_setschedparam(thread, policy, &param);
  if (status) {
    STEPIT_WARN("Failed to set thread priority (error code: {}).", status);
    return -1;
  }
  pthread_getschedparam(thread, &policy, &param);
  return param.sched_priority;
}

Communication::Communication(const std::string &robot_factory)
    : api_(RobotApi::make(robot_factory)),
      dof_{api_->getDoF()},
      freq_{api_->getCommFreq()},
      low_state_(api_->getDoF(), api_->getNumLegs()),
      low_cmd_(api_->getDoF()) {
  api_->getControl(true);
}

Communication::~Communication() {
  shutdown();
  api_->getControl(false);
}

LowState Communication::getLowState() {
  std::lock_guard<std::mutex> lock(mutex_);
  return low_state_;
}

void Communication::setLowCmd(const LowCmd &low_cmd) {
  STEPIT_ASSERT_EQ(low_cmd.size(), dof_, "Low command size mismatch.");
  std::lock_guard<std::mutex> lock(mutex_);
  low_cmd_ = low_cmd;
}

void Communication::launch() {
  if (main_loop_thread_.joinable()) {  // thread has already started
    if (communicating_) communicating_ = false;
    main_loop_thread_.join();
  }

  thread_id_        = -1;
  active_           = false;
  main_loop_thread_ = std::thread([this] { mainLoop(); });

  while (thread_id_ < 0) std::this_thread::sleep_for(USec(10));
  long comm_cpuid{-1};
  if (getenv("STEPIT_COMM_CPUID", comm_cpuid)) {
    setThreadCPU(thread_id_, comm_cpuid);
  }
  int priority = setThreadRT(main_loop_thread_.native_handle());
  if (priority > 0) STEPIT_LOG("Set communication thread priority to {}.", priority);
}

void Communication::shutdown() {
  communicating_ = false;
  connected_     = false;
  if (main_loop_thread_.joinable()) {
    main_loop_thread_.join();
    STEPIT_LOG("Disconnected from Robot.");
  }
  thread_id_ = -1;
}

void Communication::mainLoop() {
  thread_id_     = gettid();
  communicating_ = true;
  Rate rate(freq_);
  bool printed1 = false, printed2 = false;

  while (communicating_) {
    mainEvent();
    if (not printed1 and connected_) {
      STEPIT_LOG("Robot connected.");
      printed1 = printed2 = true;
    }
    if (not printed2 and not connected_) {
      STEPIT_CRIT("Robot not connected.");
      printed2 = true;
    }
    rate.sleep();
  }
}

void Communication::mainEvent() {
  api_->recv();
  {
    std::lock_guard<std::mutex> _(mutex_);
    api_->getRecv(low_state_);
    connected_ = low_state_.tick != 0;
    if (frozen_) active_ = false;
    if (not active_) {
      for (auto &motor_cmd : low_cmd_) {
        motor_cmd.q   = 0.;
        motor_cmd.dq  = 0.;
        motor_cmd.tor = 0.;
        motor_cmd.Kp  = 0.;
        motor_cmd.Kd  = spec().kd_damped_mode;
      }
    }
    api_->setSend(low_cmd_);
  }
  if (active_ or not spec().auto_damped_mode) api_->send();
  {
    std::lock_guard<std::mutex> _(mutex_);
    publisher::publishLowLevel(spec(), low_state_, low_cmd_);
  }
  tick_.fetch_add(1, std::memory_order_release);
}

void Communication::waitUntil(std::size_t tick) const {
  while (tick_.load(std::memory_order_acquire) < tick) {
    std::this_thread::sleep_for(USec(10));
  }
}
}  // namespace stepit

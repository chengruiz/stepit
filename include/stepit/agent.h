#ifndef STEPIT_AGENT_H_
#define STEPIT_AGENT_H_

#include <atomic>
#include <map>
#include <string>
#include <vector>

#include <stepit/communication.h>
#include <stepit/control_input.h>
#include <stepit/policy.h>

namespace stepit {
class Agent final {
 public:
  explicit Agent(const std::string &robot_factory, const std::vector<std::string> &control_factories);
  ~Agent() { shutdown(); }

  void addPolicy(const std::string &policy_factory, const std::string &home_dir);
  int stepit();

  enum class State : std::uint8_t {
    kFrozen,
    kResting,
    kStandingUp,
    kLyingDown,
    kStandingStill,
    kReturningToStanding,
    kPolicy,
  };

  enum class Action : std::uint8_t {
    kStandUp,
    kLieDown,
    kStandUpOrLieDown,
    kPolicyOn,
    kPolicyOff,
    kPolicyOnOrOff,
    kFreeze,
    kUnfreeze,
    kCyclePolicy,
    kSelectPolicy,
    kInvalid = 255,
  };

 private:
  static const std::map<std::string, Action> kActionMap;

  void launch();
  void shutdown();
  void mainLoop();
  void mainEvent();
  void updateControlInput();
  void handleControlRequest(ControlRequest request);
  void stepStateMachine();
  void trySwitchState(State next_state);
  void onExit(State curr_state);

  State eventStandingStill();
  State eventStandingUp();
  State eventLyingDown();
  State eventReturningToStanding();
  State eventPolicy();

  bool selectPolicy(const std::string &name);
  bool selectPolicy(std::size_t index);
  bool runPolicy();
  void publishStatus() const;

  bool isActivePolicyTrusted() const { return active_policy_ != nullptr and active_policy_->isTrusted(); }
  bool isReadyForPolicy() const {
    return curr_state_ == State::kStandingStill                            // common case
           or (curr_state_ != State::kFrozen and isActivePolicyTrusted())  // trusted policy
        ;
  }
  std::size_t calculateStepCount(double seconds) const {
    return static_cast<std::size_t>(seconds * static_cast<double>(communication_.getFreq()));
  }

  const RobotSpec &spec() const { return communication_.spec(); }
  std::size_t dof() const { return communication_.dof(); }

  Communication communication_;
  MultipleControlInputs ctrl_input_;
  bool ctrl_available_{false};

  std::thread main_loop_thread_;
  std::atomic<pid_t> thread_id_{-1};
  LowCmd low_cmd_;
  std::atomic<bool> agent_started_{false};
  State curr_state_{State::kResting};
  State next_state_{State::kResting};
  std::size_t state_tick_{};

  std::vector<Policy::Ptr> policies_;
  Policy *active_policy_{nullptr};
  std::size_t active_policy_idx_{0};
  ControlRequests policy_requests_;
  Timer policy_timer_;
};
}  // namespace stepit

#endif  // STEPIT_AGENT_H_

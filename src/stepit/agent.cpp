#include <stepit/agent.h>
#include <stepit/logging.h>
#include <stepit/spin.h>

namespace stepit {
// clang-format off
const std::map<std::string, Agent::Action> Agent::kActionMap = {
    {"StandUp",          Action::kStandUp},
    {"LieDown",          Action::kLieDown},
    {"StandUpOrLieDown", Action::kStandUpOrLieDown},
    {"PolicyOn",         Action::kPolicyOn},
    {"PolicyOff",        Action::kPolicyOff},
    {"PolicyOnOrOff",    Action::kPolicyOnOrOff},
    {"Freeze",           Action::kFreeze},
    {"Unfreeze",         Action::kUnfreeze},
    {"CyclePolicy",      Action::kCyclePolicy},
    {"SelectPolicy",     Action::kSelectPolicy},
};
// clang-format on

Agent::Agent(const std::string &robot_type, const std::vector<std::string> &ctrl_type)
    : Communication(robot_type), ctrl_input_(ctrl_type), command_(dof()) {}

void Agent::addPolicy(const std::string &policy_type, const std::string &home_dir) {
  auto policy = Policy::make(policy_type, spec(), home_dir);
  if (getCommunicationFreq() % policy->getControlFreq() != 0) {
    STEPIT_WARN("Policy control frequency ({}) is not a divisor of the communication frequency ({}).",
                policy->getControlFreq(), getCommunicationFreq());
  }
  policies_.push_back(std::move(policy));
  if (policies_.size() == 1) {
    active_policy_idx_ = 0;
    active_policy_     = policies_[0].get();
  }
}

int Agent::stepit() {
  startCommunicationThread();
  startAgentThread();
  return spin();
}

void Agent::startAgentThread() {
  if (agent_thread_.joinable()) {
    if (agent_started_) agent_started_ = false;
    agent_thread_.join();
  }
  agent_thread_ = std::thread([this] { agentMainLoop(); });

  while (agent_tid_ < 0) std::this_thread::sleep_for(USec(10));
  long agent_cpuid{-1};
  if (getenv("STEPIT_AGENT_CPUID", agent_cpuid)) {
    setThreadCPU(agent_tid_, agent_cpuid);
  }
  int priority = setThreadRT(agent_thread_.native_handle());
  if (priority > 0) STEPIT_LOG("Set agent thread priority to {}.", priority);
}

void Agent::stopAgentThread() {
  if (agent_thread_.joinable()) {
    agent_started_ = false;
    agent_thread_.join();
    agent_tid_ = -1;
    STEPIT_LOG("Agent stopped.");
  }
}

void Agent::agentMainLoop() {
  STEPIT_LOG("Agent started.");

  agent_tid_            = gettid();
  agent_started_        = true;
  ctrl_available_       = ctrl_input_.available();
  bool robot_connected  = isConnected();
  std::size_t next_tick = getCommunicationTick();

  if (ctrl_available_) {
    STEPIT_LOG("Control input is available.");
  } else {
    STEPIT_WARN("Control input is unavailable.");
  }

  while (agent_started_) {
    if (not robot_connected) {
      robot_connected = isConnected();
    } else if (not isConnected()) {
      STEPIT_CRIT("Agent quit due to interrupted communication.");
      return;
    } else {
      agentMainEvent();
    }

    publishStatus();
    waitForCommunicationTick(++next_tick);
  }
}

void Agent::agentMainEvent() {
  updateControlInput();
  stepStateMachine();
}

void Agent::updateControlInput() {
  bool ctrl_available = ctrl_input_.available();
  if (ctrl_available and not ctrl_available_) {
    STEPIT_LOG("Control input is available.");
  } else if (not ctrl_available and ctrl_available_) {
    STEPIT_WARN("Control input is unavailable.");
  }
  ctrl_available_ = ctrl_available;
  next_state_     = curr_state_;

  if (ctrl_available_) {
    ctrl_input_.poll();
    while (true) {
      boost::optional<ControlRequest> request = ctrl_input_.pop();
      if (not request.has_value()) break;
      handleControlRequest(std::move(request.value()));
    }
  }
}

void Agent::handleControlRequest(ControlRequest request) {
  bool unrecognized = false;
  if (request.channelMatches("Agent")) {
    auto action = lookupAction(request.action(), kActionMap);
    switch (action) {
      case Action::kStandUp:
        if (curr_state_ == State::kResting) {
          next_state_ = State::kStandingUp;
          request.response(kSuccess);
        } else {
          request.response(kNotInCorrectState, kAgentNotInCorrectState);
        }
        break;
      case Action::kLieDown:
        if (curr_state_ == State::kStandingStill or curr_state_ == State::kPolicy) {
          next_state_ = State::kLyingDown;
          request.response(kSuccess);
        } else {
          request.response(kNotInCorrectState, kAgentNotInCorrectState);
        }
        break;
      case Action::kStandUpOrLieDown:
        if (curr_state_ == State::kResting) {
          next_state_ = State::kStandingUp;
          request.response(kSuccess);
        } else if (curr_state_ == State::kStandingStill or curr_state_ == State::kPolicy) {
          next_state_ = State::kLyingDown;
          request.response(kSuccess);
        } else {
          request.response(kNotInCorrectState, kAgentNotInCorrectState);
        }
        break;
      case Action::kPolicyOn:
        if (active_policy_ == nullptr) {
          request.response(kPolicyNotFound, "No policy available.");
        } else if (isReadyForPolicy()) {
          next_state_ = State::kPolicy;
          request.response(kSuccess);
        } else {
          request.response(kNotInCorrectState, kAgentNotInCorrectState);
        }
        break;
      case Action::kPolicyOff:
        if (curr_state_ == State::kPolicy) {
          next_state_ = State::kReturningToStanding;
          request.response(kSuccess);
        } else {
          request.response(kNotInCorrectState, kAgentNotInCorrectState);
        }
        break;
      case Action::kPolicyOnOrOff:
        if (curr_state_ == State::kPolicy) {
          next_state_ = State::kReturningToStanding;
          request.response(kSuccess);
        } else if (active_policy_ == nullptr) {
          request.response(kPolicyNotFound, "No policy available.");
        } else if (isReadyForPolicy()) {
          next_state_ = State::kPolicy;
          request.response(kSuccess);
        } else {
          request.response(kNotInCorrectState, kAgentNotInCorrectState);
        }
        break;
      case Action::kFreeze:
        next_state_ = State::kFrozen;
        request.response(kSuccess);
        break;
      case Action::kUnfreeze:
        if (curr_state_ == State::kFrozen) {
          next_state_ = State::kResting;
          request.response(kSuccess);
        } else {
          request.response(kNotInCorrectState, kAgentNotInCorrectState);
        }
        break;
      case Action::kCyclePolicy:
        if (not policies_.empty()) {
          selectPolicy((active_policy_idx_ + 1) % policies_.size());
          request.response(kSuccess);
        } else {
          request.response(kPolicyNotFound, "No policy available.");
        }
        break;
      case Action::kSelectPolicy:
        if (selectPolicy(request.argument())) {
          request.response();
        } else {
          request.response(kPolicyNotFound, fmt::format("Policy '{}' not found.", request.argument()));
        }
        break;
      default:
        unrecognized = true;
    }
  } else if (request.channelMatches("Policy")) {
    if (curr_state_ == State::kPolicy) {
      policy_requests_.push_back(std::move(request));
    } else {
      request.response(kNotInCorrectState);
    }
  } else {
    unrecognized = true;
  }

  if (unrecognized) {
    request.response(kUnrecognizedRequest);
  }
}

void Agent::stepStateMachine() {
  if (next_state_ != curr_state_) {
    trySwitchState(next_state_);
  }

  State next_state = curr_state_;
  switch (curr_state_) {
    case State::kFrozen:
      if (state_tick_ == 0) STEPIT_LOG("Agent frozen.");
      setFrozen();
      break;
    case State::kResting:
      setActive(false);
      break;
    case State::kStandingStill:
      next_state = eventStandingStill();
      break;
    case State::kPolicy:
      next_state = eventPolicy();
      break;
    case State::kStandingUp:  // from lying to standing
      next_state = eventStandingUp();
      break;
    case State::kLyingDown:  // from standing / moving to lying
      next_state = eventLyingDown();
      break;
    case State::kReturningToStanding:  // from policy to standing
      next_state = eventReturningToStanding();
      break;
    default:
      STEPIT_UNREACHABLE();
      break;
  }

  trySwitchState(next_state);
}

void Agent::trySwitchState(State next_state) {
  if (next_state > State::kResting and not(next_state == State::kPolicy and isActivePolicyTrusted())) {
    if (std::abs(low_state_msg_.imu.rpy[0]) > spec().safety.roll or
        std::abs(low_state_msg_.imu.rpy[1]) > spec().safety.pitch) {
      next_state = State::kFrozen;
      STEPIT_WARN("Agent froze due to safety violations.");
    }
  }

  if (curr_state_ == next_state) {
    ++state_tick_;
    return;
  }

  onExit(curr_state_);
  curr_state_ = next_state;
  state_tick_ = 0;
}

void Agent::onExit(State curr_state) {
  switch (curr_state) {
    case State::kFrozen:
      STEPIT_LOG("Agent unfroze.");
      setFrozen(false);
      break;
    case State::kPolicy:
      active_policy_->exit();
      if (policy_timer_.count() > 0) {
        STEPIT_LOG("Average policy time: {}.", policy_timer_.mean<USec>());
        policy_timer_.clear();
      }
      break;
    default:
      break;
  }
}

Agent::State Agent::eventStandingStill() {
  if (state_tick_ == 0) {
    STEPIT_LOG("Standing.");
  }

  for (std::size_t i{}; i < dof(); ++i) {
    command_[i].q   = spec().standing_cfg[i];
    command_[i].dq  = 0.0F;
    command_[i].tor = 0.0F;
    command_[i].Kp  = spec().kp[i];
    command_[i].Kd  = spec().kd[i];
  }
  applyCommand();
  return State::kStandingStill;
}

Agent::State Agent::eventStandingUp() {
  std::vector<float> joint_pos(dof());
  {
    std::lock_guard<std::mutex> _(comm_mtx_);
    for (std::size_t i{}; i < dof(); ++i) {
      joint_pos[i] = low_state_msg_.motor_state[i].q;
    }
  }

  if (state_tick_ == 0) {
    STEPIT_LOG("Standing up...");
    for (std::size_t i{}; i < dof(); ++i) {
      command_[i].q   = joint_pos[i];
      command_[i].dq  = 0.0F;
      command_[i].tor = 0.0F;
      command_[i].Kp  = spec().kp[i];
      command_[i].Kd  = spec().kd[i];
    }
    setActive();
  }
  for (std::size_t i{}; i < dof(); ++i) {
    float stuck_threshold = spec().stuck_threshold[i];
    if (stuck_threshold > 0 and std::abs(command_[i].q - joint_pos[i]) > stuck_threshold) {
      STEPIT_WARN("Failed to stand up as legs are probably stuck.");
      return State::kResting;
    }
  }

  std::size_t num_steps1 = getNumSteps(spec().resetting_time);
  std::size_t num_steps2 = getNumSteps(spec().standing_up_time);
  if (state_tick_ < num_steps1) {
    float factor = 1.0F / static_cast<float>(num_steps1 - state_tick_);
    for (std::size_t i{}; i < dof(); ++i) {
      command_[i].q   = lerp(command_[i].q, spec().lying_cfg[i], factor);
      command_[i].dq  = 0.0F;
      command_[i].tor = 0.0F;
      command_[i].Kp  = spec().kp[i];
      command_[i].Kd  = spec().kd[i];
    }
    applyCommand();
    return State::kStandingUp;
  }

  std::size_t tick2 = state_tick_ - num_steps1;
  float factor      = 1.0F / static_cast<float>(num_steps2 - tick2 + 1);
  for (std::size_t i{}; i < dof(); ++i) {
    command_[i].q   = lerp(command_[i].q, spec().standing_cfg[i], factor);
    command_[i].dq  = 0.0F;
    command_[i].tor = 0.0F;
    command_[i].Kp  = spec().kp[i];
    command_[i].Kd  = spec().kd[i];
  }
  applyCommand();

  if (tick2 < num_steps2) return State::kStandingUp;
  return State::kStandingStill;
}

Agent::State Agent::eventLyingDown() {
  if (state_tick_ == 0) {
    STEPIT_LOG("Lying down...");
    std::lock_guard<std::mutex> _(comm_mtx_);
    for (std::size_t i{}; i < dof(); ++i) {
      command_[i].q = low_state_msg_.motor_state[i].q;
    }
  }
  std::size_t num_steps = getNumSteps(spec().lying_down_time);
  float factor          = 1.0F / static_cast<float>(num_steps - state_tick_ + 1);
  for (std::size_t i{}; i < dof(); ++i) {
    command_[i].q   = lerp(command_[i].q, spec().lying_cfg[i], factor);
    command_[i].dq  = 0.0F;
    command_[i].tor = 0.0F;
    command_[i].Kp  = spec().kp[i];
    command_[i].Kd  = spec().kd[i];
  }
  applyCommand();

  if (state_tick_ < num_steps) return State::kLyingDown;
  STEPIT_LOG("Lying.");
  return State::kResting;
}

Agent::State Agent::eventReturningToStanding() {
  if (state_tick_ == 0) {
    STEPIT_LOG("Policy quit.");
    std::lock_guard<std::mutex> lock(comm_mtx_);
    for (std::size_t i{}; i < dof(); ++i) {
      command_[i].q   = low_state_msg_.motor_state[i].q;
      command_[i].dq  = 0.0F;
      command_[i].tor = 0.0F;
    }
  }
  std::size_t num_steps = getNumSteps(spec().returning_to_standing_time);
  float factor          = 1.0F / static_cast<float>(num_steps - state_tick_ + 1);
  for (std::size_t i{}; i < dof(); ++i) {
    command_[i].q   = lerp(command_[i].q, spec().standing_cfg[i], factor);
    command_[i].dq  = 0.0F;
    command_[i].tor = 0.0F;
    command_[i].Kp  = spec().kp[i];
    command_[i].Kd  = spec().kd[i];
  }
  applyCommand();

  if (state_tick_ < num_steps) return State::kReturningToStanding;
  return State::kStandingStill;
}

Agent::State Agent::eventPolicy() {
  if (state_tick_ == 0) {
    if (not active_policy_->reset()) {
      STEPIT_CRIT("Failed to initialize the policy.");
      return State::kResting;
    }
    policy_timer_.clear();
    setActive();
    STEPIT_LOG("Policy started.");
  }
  std::size_t ratio = std::max(
      static_cast<std::size_t>(std::round(static_cast<double>(getCommunicationFreq()) /
                                          static_cast<double>(active_policy_->getControlFreq()))),
      1UL);
  if (state_tick_ % ratio == 0) {
    bool status = runPolicy();
    if (not status) {
      STEPIT_CRIT("Policy runtime failure.");
      return State::kResting;
    }
  }
  return State::kPolicy;
}

bool Agent::selectPolicy(const std::string &name) {
  for (std::size_t i{}; i < policies_.size(); ++i) {
    if (policies_[i]->getName() == name) return selectPolicy(i);
  }
  return false;
}

bool Agent::selectPolicy(std::size_t index) {
  if (index >= policies_.size()) return false;
  active_policy_idx_ = index;
  active_policy_     = policies_[index].get();
  if (curr_state_ == State::kPolicy) {
    if (policy_timer_.count() > 0) {
      STEPIT_LOG("Average policy time: {}.", policy_timer_.mean<USec>());
      policy_timer_.clear();
    }
    active_policy_->reset();
  }
  STEPIT_LOG("Switch current policy to '{}'.", active_policy_->getName());
  return true;
}

bool Agent::runPolicy() {
  TimerContext _(policy_timer_);
  LowState low_state{getLowState()};

  if (active_policy_->act(low_state, policy_requests_, command_)) {
    for (auto &&request : policy_requests_) {
      request.response(kUnrecognizedRequest);
    }
    policy_requests_.clear();
    applyCommand();
    return true;
  }

  for (std::size_t i{}; i < dof(); ++i) {
    command_[i].q   = spec().standing_cfg[i];
    command_[i].dq  = 0.0F;
    command_[i].tor = 0.0F;
    command_[i].Kp  = spec().kp[i];
    command_[i].Kd  = spec().kd[i];
  }
  applyCommand();
  return false;
}

void Agent::applyCommand() {
  std::lock_guard<std::mutex> lock(comm_mtx_);
  low_cmd_msg_ = command_;
}

void Agent::publishStatus() const {
  publisher::updateStatus("Agent/ActivePolicy", active_policy_ ? active_policy_->getName() : "null");
  publisher::updateStatus("Agent/Connected", isConnected());
  publisher::updateStatus("Agent/ControlAvailable", ctrl_available_);
  publisher::updateStatus("Agent/State", static_cast<std::uint8_t>(curr_state_));
  publisher::publishStatus();
}
}  // namespace stepit

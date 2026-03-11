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

Agent::Agent(const std::string &robot_factory, const std::vector<std::string> &control_factories)
    : communication_(robot_factory), ctrl_input_(control_factories), low_cmd_(communication_.dof()) {}

void Agent::addPolicy(const std::string &policy_factory, const std::string &home_dir) {
  auto policy = Policy::make(policy_factory, spec(), home_dir);
  if (communication_.getFreq() % policy->getControlFreq() != 0) {
    STEPIT_WARN("Policy control frequency ({}) is not a divisor of the communication frequency ({}).",
                policy->getControlFreq(), communication_.getFreq());
  }
  policies_.push_back(std::move(policy));
  if (policies_.size() == 1) {
    active_policy_idx_ = 0;
    active_policy_     = policies_[0].get();
  }
}

int Agent::stepit() {
  communication_.launch();
  launch();
  return spin();
}

void Agent::launch() {
  if (main_loop_thread_.joinable()) {
    if (agent_started_) agent_started_ = false;
    main_loop_thread_.join();
  }
  thread_id_        = -1;
  main_loop_thread_ = std::thread([this] { mainLoop(); });

  while (thread_id_ < 0) std::this_thread::sleep_for(USec(10));
  long agent_cpuid{-1};
  if (getenv("STEPIT_AGENT_CPUID", agent_cpuid)) {
    setThreadCPU(thread_id_, agent_cpuid);
  }
  int priority = setThreadRT(main_loop_thread_.native_handle());
  if (priority > 0) STEPIT_LOG("Set agent thread priority to {}.", priority);
}

void Agent::shutdown() {
  if (main_loop_thread_.joinable()) {
    agent_started_ = false;
    main_loop_thread_.join();
    thread_id_ = -1;
    STEPIT_LOG("Agent stopped.");
  }
}

void Agent::mainLoop() {
  STEPIT_LOG("Agent started.");

  thread_id_            = gettid();
  agent_started_        = true;
  ctrl_available_       = ctrl_input_.available();
  bool robot_connected  = communication_.isConnected();
  std::size_t next_tick = communication_.getTick();

  if (ctrl_available_) {
    STEPIT_LOG("Control input is available.");
  } else {
    STEPIT_WARN("Control input is unavailable.");
  }

  while (agent_started_) {
    if (not robot_connected) {
      robot_connected = communication_.isConnected();
    } else if (not communication_.isConnected()) {
      STEPIT_CRIT("Agent quit due to interrupted communication.");
      return;
    } else {
      mainEvent();
    }

    publishStatus();
    communication_.waitUntil(++next_tick);
  }
}

void Agent::mainEvent() {
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
      communication_.setFrozen();
      break;
    case State::kResting:
      communication_.setActive(false);
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
    LowState low_state{communication_.getLowState()};
    if (std::abs(low_state.imu.rpy[0]) > spec().safety.roll or std::abs(low_state.imu.rpy[1]) > spec().safety.pitch) {
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
      communication_.setFrozen(false);
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
    low_cmd_[i].q   = spec().standing_cfg[i];
    low_cmd_[i].dq  = 0.0F;
    low_cmd_[i].tor = 0.0F;
    low_cmd_[i].Kp  = spec().kp[i];
    low_cmd_[i].Kd  = spec().kd[i];
  }
  communication_.setLowCmd(low_cmd_);
  return State::kStandingStill;
}

Agent::State Agent::eventStandingUp() {
  LowState low_state{communication_.getLowState()};
  std::vector<float> joint_pos(dof());
  for (std::size_t i{}; i < dof(); ++i) {
    joint_pos[i] = low_state.motor_state[i].q;
  }

  if (state_tick_ == 0) {
    STEPIT_LOG("Standing up...");
    for (std::size_t i{}; i < dof(); ++i) {
      low_cmd_[i].q   = joint_pos[i];
      low_cmd_[i].dq  = 0.0F;
      low_cmd_[i].tor = 0.0F;
      low_cmd_[i].Kp  = spec().kp[i];
      low_cmd_[i].Kd  = spec().kd[i];
    }
    communication_.setActive();
  }
  for (std::size_t i{}; i < dof(); ++i) {
    float stuck_threshold = spec().stuck_threshold[i];
    if (stuck_threshold > 0 and std::abs(low_cmd_[i].q - joint_pos[i]) > stuck_threshold) {
      STEPIT_WARN("Failed to stand up as legs are probably stuck.");
      return State::kResting;
    }
  }

  std::size_t num_steps1 = calculateStepCount(spec().resetting_time);
  std::size_t num_steps2 = calculateStepCount(spec().standing_up_time);
  if (state_tick_ < num_steps1) {
    float factor = 1.0F / static_cast<float>(num_steps1 - state_tick_);
    for (std::size_t i{}; i < dof(); ++i) {
      low_cmd_[i].q   = lerp(low_cmd_[i].q, spec().lying_cfg[i], factor);
      low_cmd_[i].dq  = 0.0F;
      low_cmd_[i].tor = 0.0F;
      low_cmd_[i].Kp  = spec().kp[i];
      low_cmd_[i].Kd  = spec().kd[i];
    }
    communication_.setLowCmd(low_cmd_);
    return State::kStandingUp;
  }

  std::size_t tick2 = state_tick_ - num_steps1;
  float factor      = 1.0F / static_cast<float>(num_steps2 - tick2 + 1);
  for (std::size_t i{}; i < dof(); ++i) {
    low_cmd_[i].q   = lerp(low_cmd_[i].q, spec().standing_cfg[i], factor);
    low_cmd_[i].dq  = 0.0F;
    low_cmd_[i].tor = 0.0F;
    low_cmd_[i].Kp  = spec().kp[i];
    low_cmd_[i].Kd  = spec().kd[i];
  }
  communication_.setLowCmd(low_cmd_);

  if (tick2 < num_steps2) return State::kStandingUp;
  return State::kStandingStill;
}

Agent::State Agent::eventLyingDown() {
  if (state_tick_ == 0) {
    STEPIT_LOG("Lying down...");
    LowState low_state{communication_.getLowState()};
    for (std::size_t i{}; i < dof(); ++i) {
      low_cmd_[i].q = low_state.motor_state[i].q;
    }
  }
  std::size_t num_steps = calculateStepCount(spec().lying_down_time);
  float factor          = 1.0F / static_cast<float>(num_steps - state_tick_ + 1);
  for (std::size_t i{}; i < dof(); ++i) {
    low_cmd_[i].q   = lerp(low_cmd_[i].q, spec().lying_cfg[i], factor);
    low_cmd_[i].dq  = 0.0F;
    low_cmd_[i].tor = 0.0F;
    low_cmd_[i].Kp  = spec().kp[i];
    low_cmd_[i].Kd  = spec().kd[i];
  }
  communication_.setLowCmd(low_cmd_);

  if (state_tick_ < num_steps) return State::kLyingDown;
  STEPIT_LOG("Lying.");
  return State::kResting;
}

Agent::State Agent::eventReturningToStanding() {
  if (state_tick_ == 0) {
    STEPIT_LOG("Policy quit.");
    LowState low_state{communication_.getLowState()};
    for (std::size_t i{}; i < dof(); ++i) {
      low_cmd_[i].q   = low_state.motor_state[i].q;
      low_cmd_[i].dq  = 0.0F;
      low_cmd_[i].tor = 0.0F;
    }
  }
  std::size_t num_steps = calculateStepCount(spec().returning_to_standing_time);
  float factor          = 1.0F / static_cast<float>(num_steps - state_tick_ + 1);
  for (std::size_t i{}; i < dof(); ++i) {
    low_cmd_[i].q   = lerp(low_cmd_[i].q, spec().standing_cfg[i], factor);
    low_cmd_[i].dq  = 0.0F;
    low_cmd_[i].tor = 0.0F;
    low_cmd_[i].Kp  = spec().kp[i];
    low_cmd_[i].Kd  = spec().kd[i];
  }
  communication_.setLowCmd(low_cmd_);

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
    communication_.setActive();
    STEPIT_LOG("Policy '{}' started.", active_policy_->getName());
  }
  std::size_t ratio = std::max(
      static_cast<std::size_t>(std::round(static_cast<double>(communication_.getFreq()) /
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
  if (active_policy_idx_ == index) return true;
  if (curr_state_ == State::kPolicy) {
    onExit(State::kPolicy);
    state_tick_ = 0;
  }
  active_policy_idx_ = index;
  active_policy_     = policies_[index].get();
  STEPIT_LOG("Switch current policy to '{}'.", active_policy_->getName());
  return true;
}

bool Agent::runPolicy() {
  TimerContext _(policy_timer_);
  LowState low_state{communication_.getLowState()};

  if (active_policy_->act(low_state, policy_requests_, low_cmd_)) {
    for (auto &&request : policy_requests_) {
      request.response(kUnrecognizedRequest);
    }
    policy_requests_.clear();
    communication_.setLowCmd(low_cmd_);
    return true;
  }

  for (std::size_t i{}; i < dof(); ++i) {
    low_cmd_[i].q   = spec().standing_cfg[i];
    low_cmd_[i].dq  = 0.0F;
    low_cmd_[i].tor = 0.0F;
    low_cmd_[i].Kp  = spec().kp[i];
    low_cmd_[i].Kd  = spec().kd[i];
  }
  communication_.setLowCmd(low_cmd_);
  return false;
}

void Agent::publishStatus() const {
  publisher::updateStatus("Agent/ActivePolicy", active_policy_ ? active_policy_->getName() : "null");
  publisher::updateStatus("Agent/Connected", communication_.isConnected());
  publisher::updateStatus("Agent/ControlAvailable", ctrl_available_);
  publisher::updateStatus("Agent/State", static_cast<std::uint8_t>(curr_state_));
  publisher::publishStatus();
}
}  // namespace stepit

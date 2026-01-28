#include <stepit/joystick/joystick.h>

namespace stepit {
namespace joystick {
std::map<std::size_t, JoystickControl::Rule> JoystickControl::rules_;
std::size_t JoystickControl::rule_count_{0};
std::mutex JoystickControl::rule_mutex_;

JoystickControl::JoystickControl(const std::string &js_type) { js_ = Joystick::make(js_type); }

void JoystickControl::poll() {
  js_->getState(state_);
  if ((state_.LAS().pressed and state_.RAS().on_press) or (state_.LAS().on_press and state_.RAS().pressed)) {
    put("Agent/Freeze");
    return;
  }

  if (state_.lt() > 0.9) {
    if (state_.X().on_press) {
      put("Agent/Unfreeze");
    } else if (state_.A().on_press) {
      put("Agent/StandUpOrLieDown");
    } else if (state_.B().on_press) {
      put("Agent/PolicyOnOrOff");
    } else if (state_.Y().on_press) {
      put("Agent/CyclePolicy");
    }
  }

  std::lock_guard<std::mutex> lock(rule_mutex_);
  for (const auto &rule : rules_) {
    auto result = rule.second(state_);
    if (not result.empty()) put(result);
  }
}

JoystickControl::Registration::Registration(Rule rule) {
  std::lock_guard<std::mutex> lock(rule_mutex_);
  rule_id_         = rule_count_++;
  rules_[rule_id_] = std::move(rule);
}

JoystickControl::Registration::~Registration() {
  std::lock_guard<std::mutex> lock(rule_mutex_);
  if (rule_id_ != static_cast<std::size_t>(-1)) rules_.erase(rule_id_);
}

JoystickControl::Registration::Registration(Registration &&other) noexcept {
  std::lock_guard<std::mutex> lock(rule_mutex_);
  rule_id_       = other.rule_id_;
  other.rule_id_ = static_cast<std::size_t>(-1);
}

JoystickControl::Registration &JoystickControl::Registration::operator=(Registration &&other) noexcept {
  if (this != &other) {
    std::lock_guard<std::mutex> lock(rule_mutex_);
    if (rule_id_ != static_cast<std::size_t>(-1)) rules_.erase(rule_id_);
    rule_id_       = other.rule_id_;
    other.rule_id_ = static_cast<std::size_t>(-1);
  }
  return *this;
}
}  // namespace joystick

STEPIT_REGISTER_CTRLINPUT(joystick, kDefPriority, ControlInput::make<JoystickControl>);
}  // namespace stepit

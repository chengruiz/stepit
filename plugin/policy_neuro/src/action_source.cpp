#include <stepit/registry.h>
#include <stepit/policy_neuro/action_source.h>

namespace stepit {
namespace neuro_policy {
ActionHistory::ActionHistory(const PolicySpec &, const std::string &home_dir) {
  auto policy_cfg = yml::loadFile(home_dir + "/policy.yml");
  yml::setTo(policy_cfg, "action_mean", action_mean_);
  action_his_.allocate(5);

  action_id_    = getFieldId("action");
  action_p1_id_ = registerProvision("action_p1", 0);
  action_p2_id_ = registerProvision("action_p2", 0);
}

void ActionHistory::initFieldProperties() {
  auto action_dim = getFieldSize(action_id_);
  setFieldSize(action_p1_id_, action_dim);
  setFieldSize(action_p2_id_, action_dim);
}

bool ActionHistory::reset() {
  action_his_.clear();
  return true;
}

bool ActionHistory::update(const LowState &low_state, ControlRequests &requests, FieldMap &result) {
  result[action_p1_id_] = action_his_.get(-1, action_mean_);
  result[action_p2_id_] = action_his_.get(-2, action_mean_);
  return true;
}

void ActionHistory::postUpdate(const FieldMap &field_map) { action_his_.push_back(field_map.at(action_id_)); }

ActionFilter::ActionFilter(const PolicySpec &, const std::string &home_dir) {
  auto policy_cfg = yml::loadFile(home_dir + "/policy.yml");
  yml::setTo(policy_cfg, "action_mean", action_mean_);
  auto config = yml::loadFile(home_dir + "/action_filter.yml");
  yml::setTo(config, "window_size", window_size_);
  action_his_.allocate(window_size_);
  STEPIT_ASSERT(window_size_ > 1, "'window_size' must be greater than 1.");
  STEPIT_LOGNT("Action low-pass filter is applied with a window size of {}.", window_size_);
  action_id_ = registerRequirement("action");
}

bool ActionFilter::reset() {
  action_his_.clear();
  return true;
}

bool ActionFilter::update(const LowState &low_state, ControlRequests &requests, FieldMap &result) {
  auto &action = result.at(action_id_);
  action_his_.push_back(action);
  for (int i{1}; i < window_size_; ++i) {
    action += action_his_.get(-i - 1, action_mean_);
  }
  action /= static_cast<float>(window_size_);
  return true;
}

STEPIT_REGISTER_MODULE(action_history, kDefPriority, Module::make<ActionHistory>);
STEPIT_REGISTER_MODULE(action_filter, kDefPriority, Module::make<ActionFilter>);
STEPIT_REGISTER_FIELD_SOURCE(action_p1, kDefPriority, Module::make<ActionHistory>);
STEPIT_REGISTER_FIELD_SOURCE(action_p2, kDefPriority, Module::make<ActionHistory>);
}  // namespace neuro_policy
}  // namespace stepit

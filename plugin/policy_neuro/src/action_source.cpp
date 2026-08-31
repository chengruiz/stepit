#include <stepit/registry.h>
#include <stepit/policy_neuro/action_source.h>

namespace stepit {
namespace neuro_policy {
ActionHistory::ActionHistory(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : Module(policy_spec, ModuleSpec(module_spec, "action_history")) {
  default_action_ = policy_spec.default_action;
  action_buf_.allocate(5);

  action_id_      = getFieldId("action");
  last_action_id_ = registerProvision("last_action", DataType::kFloat32);  // alias of action_p1
  action_p1_id_   = registerProvision("action_p1", DataType::kFloat32);
  action_p2_id_   = registerProvision("action_p2", DataType::kFloat32);
}

bool ActionHistory::init() {
  auto action_dim = getFieldSize(action_id_);

  populateArray(default_action_, action_dim);
  setFieldSpec(last_action_id_, FieldSpec{DataType::kFloat32, action_dim});
  setFieldSpec(action_p1_id_, FieldSpec{DataType::kFloat32, action_dim});
  setFieldSpec(action_p2_id_, FieldSpec{DataType::kFloat32, action_dim});
  return true;
}

bool ActionHistory::reset() {
  action_buf_.clear();
  return true;
}

bool ActionHistory::update(const LowState &low_state, ControlRequests &requests, FieldMap &context) {
  context[last_action_id_] = action_buf_.at(-1, default_action_);
  context[action_p1_id_]   = action_buf_.at(-1, default_action_);
  context[action_p2_id_]   = action_buf_.at(-2, default_action_);
  return true;
}

void ActionHistory::postStep(const FieldMap &context) { action_buf_.push_back(context.at(action_id_).view<float>()); }

ActionFilter::ActionFilter(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : Module(policy_spec, ModuleSpec(module_spec, "action_filter")) {
  auto window_size_node = config_["window_size"];
  window_size_node.to(window_size_);
  window_size_node.require(window_size_ > 1, "'window_size' must be greater than 1");
  STEPIT_LOGNT("Action low-pass filter is applied with a window size of {}.", window_size_);

  default_action_ = policy_spec.default_action;
  action_buf_.allocate(window_size_);
  action_id_ = registerRequirement("action", DataType::kFloat32);
}

bool ActionFilter::init() {
  auto action_dim = getFieldSize(action_id_);
  populateArray(default_action_, action_dim);
  return true;
}

bool ActionFilter::reset() {
  action_buf_.clear();
  return true;
}

bool ActionFilter::update(const LowState &low_state, ControlRequests &requests, FieldMap &context) {
  auto action = context.at(action_id_).view<float>();
  action_buf_.push_back(action);
  for (int i{1}; i < window_size_; ++i) {
    action += action_buf_.at(-i - 1, default_action_);
  }
  action /= static_cast<float>(window_size_);
  return true;
}

STEPIT_REGISTER_MODULE(action_history, kDefPriority, Module::make<ActionHistory>);
STEPIT_REGISTER_MODULE(action_filter, kDefPriority, Module::make<ActionFilter>);
STEPIT_REGISTER_FIELD_SOURCE(last_action, kDefPriority, Module::make<ActionHistory>);
STEPIT_REGISTER_FIELD_SOURCE(action_p1, kDefPriority, Module::make<ActionHistory>);
STEPIT_REGISTER_FIELD_SOURCE(action_p2, kDefPriority, Module::make<ActionHistory>);
}  // namespace neuro_policy
}  // namespace stepit

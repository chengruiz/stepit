#include <stepit/modular_policy/time_source.h>

namespace stepit {
namespace modular_policy {
TimeSource::TimeSource(const ModularPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : Module(policy_spec, ModuleSpec(module_spec, "step_count")) {
  timestep_       = 1.0F / static_cast<float>(policy_spec.control_freq);
  step_count_id_  = registerProvision("step_count", DataType::kInt64, 1);
  policy_time_id_ = registerProvision("policy_time", DataType::kFloat32, 1);
}

bool TimeSource::reset() {
  step_count_ = 0;
  return true;
}

bool TimeSource::update(const LowState &, ControlRequests &, FieldMap &context) {
  context[step_count_id_]  = FieldArray<std::int64_t>::Constant(1, step_count_);
  context[policy_time_id_] = Arr1f{static_cast<float>(step_count_) * timestep_};
  ++step_count_;
  return true;
}

STEPIT_REGISTER_MODULE(time_source, kDefPriority, Module::make<TimeSource>);
STEPIT_REGISTER_FIELD_SOURCE(step_count, kDefPriority, Module::make<TimeSource>);
STEPIT_REGISTER_FIELD_SOURCE(policy_time, kDefPriority, Module::make<TimeSource>);
}  // namespace modular_policy
}  // namespace stepit

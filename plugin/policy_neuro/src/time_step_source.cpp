#include <stepit/policy_neuro/time_step_source.h>

namespace stepit {
namespace neuro_policy {
TimeStepSource::TimeStepSource(const PolicySpec &, const std::string &) {
  time_step_id_ = registerProvision("time_step", 1);
}

bool TimeStepSource::reset() {
  step_index_ = 0;
  return true;
}

bool TimeStepSource::update(const LowState &, ControlRequests &, FieldMap &context) {
  context[time_step_id_] = Arr1f{static_cast<float>(step_index_)};
  ++step_index_;
  return true;
}

STEPIT_REGISTER_MODULE(time_step, kDefPriority, Module::make<TimeStepSource>);
STEPIT_REGISTER_FIELD_SOURCE(time_step, kDefPriority, Module::make<TimeStepSource>);
}  // namespace neuro_policy
}  // namespace stepit

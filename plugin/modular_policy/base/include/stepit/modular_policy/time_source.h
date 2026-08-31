#ifndef STEPIT_MODULAR_POLICY_TIME_SOURCE_H_
#define STEPIT_MODULAR_POLICY_TIME_SOURCE_H_

#include <cstdint>

#include <stepit/modular_policy/module.h>

namespace stepit {
namespace modular_policy {
class TimeSource : public Module {
 public:
  TimeSource(const ModularPolicySpec &, const ModuleSpec &module_spec);
  bool reset() override;
  bool update(const LowState &, ControlRequests &, FieldMap &context) override;

 private:
  FieldId step_count_id_{};
  FieldId policy_time_id_{};
  float timestep_{};

  std::int64_t step_count_{};
};
}  // namespace modular_policy
}  // namespace stepit

#endif  // STEPIT_MODULAR_POLICY_TIME_SOURCE_H_

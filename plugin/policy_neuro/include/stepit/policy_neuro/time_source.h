#ifndef STEPIT_NEURO_POLICY_TIME_SOURCE_H_
#define STEPIT_NEURO_POLICY_TIME_SOURCE_H_

#include <cstdint>

#include <stepit/policy_neuro/module.h>

namespace stepit {
namespace neuro_policy {
class TimeSource : public Module {
 public:
  TimeSource(const NeuroPolicySpec &, const ModuleSpec &module_spec);
  bool reset() override;
  bool update(const LowState &, ControlRequests &, FieldMap &context) override;

 private:
  FieldId step_count_id_{};
  FieldId policy_time_id_{};
  float timestep_{};

  std::int64_t step_count_{};
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_TIME_SOURCE_H_

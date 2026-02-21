#ifndef STEPIT_NEURO_POLICY_TIME_STEP_SOURCE_H_
#define STEPIT_NEURO_POLICY_TIME_STEP_SOURCE_H_

#include <stepit/policy_neuro/field.h>

namespace stepit {
namespace neuro_policy {
class TimeStepSource : public Module {
 public:
  TimeStepSource(const PolicySpec &, const std::string &);
  bool reset() override;
  bool update(const LowState &, ControlRequests &, FieldMap &context) override;

 private:
  FieldId time_step_id_{};
  std::uint64_t step_index_{};
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_TIME_STEP_SOURCE_H_

#ifndef STEPIT_NEURO_POLICY_ROLL_PITCH_SOURCE_H_
#define STEPIT_NEURO_POLICY_ROLL_PITCH_SOURCE_H_

#include <stepit/policy_neuro/field.h>

namespace stepit {
namespace neuro_policy {
class RollPitchSource : public Module {
 public:
  RollPitchSource(const PolicySpec &policy_spec, const std::string &home_dir);
  bool reset() override { return true; }
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &result) override;

 private:
  FieldId roll_pitch_id_{};
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_ROLL_PITCH_SOURCE_H_

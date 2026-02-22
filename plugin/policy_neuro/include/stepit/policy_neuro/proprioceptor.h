#ifndef STEPIT_NEURO_POLICY_PROPRIOCEPTOR_H_
#define STEPIT_NEURO_POLICY_PROPRIOCEPTOR_H_

#include <stepit/policy_neuro/field.h>

namespace stepit {
namespace neuro_policy {
class Proprioceptor : public Module {
 public:
  Proprioceptor(const NeuroPolicySpec &policy_spec, const std::string &name);
  bool reset() override { return true; }
  bool update(const LowState &low_state, ControlRequests &, FieldMap &context) override;

 private:
  FieldId ang_vel_id_{};
  FieldId gravity_id_{};
  FieldId joint_pos_id_{};
  FieldId joint_vel_id_{};
  FieldId lin_acc_id_{};
};

class RollPitchSource : public Module {
 public:
  RollPitchSource(const NeuroPolicySpec &, const std::string &name);
  bool reset() override { return true; }
  bool update(const LowState &, ControlRequests &, FieldMap &context) override;

 private:
  FieldId roll_pitch_id_{};
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_PROPRIOCEPTOR_H_

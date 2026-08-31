#ifndef STEPIT_MODULAR_POLICY_LOCOMOTION_PROPRIOCEPTOR_H_
#define STEPIT_MODULAR_POLICY_LOCOMOTION_PROPRIOCEPTOR_H_

#include <stepit/modular_policy/module.h>

namespace stepit {
namespace modular_policy {
class Proprioceptor : public Module {
 public:
  Proprioceptor(const ModularPolicySpec &policy_spec, const ModuleSpec &module_spec);
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
  RollPitchSource(const ModularPolicySpec &, const ModuleSpec &module_spec);
  bool reset() override { return true; }
  bool update(const LowState &, ControlRequests &, FieldMap &context) override;

 private:
  FieldId roll_pitch_id_{};
};
}  // namespace modular_policy
}  // namespace stepit

#endif  // STEPIT_MODULAR_POLICY_LOCOMOTION_PROPRIOCEPTOR_H_

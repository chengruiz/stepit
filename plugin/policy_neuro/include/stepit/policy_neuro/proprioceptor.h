#ifndef STEPIT_NEURO_POLICY_PROPRIOCEPTOR_H_
#define STEPIT_NEURO_POLICY_PROPRIOCEPTOR_H_

#include <stepit/policy_neuro/field.h>

namespace stepit {
namespace neuro_policy {
class Proprioceptor : public Module {
 public:
  Proprioceptor(const PolicySpec &policy_spec, const std::string &home_dir);
  bool reset() override { return true; }
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &result) override;

 private:
  FieldId ang_vel_id_{};
  FieldId gravity_id_{};
  FieldId joint_pos_id_{};
  FieldId joint_vel_id_{};
  FieldId lin_acc_id_{};
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_PROPRIOCEPTOR_H_

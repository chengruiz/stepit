#ifndef STEPIT_MODULAR_POLICY_LOCOMOTION_ODOMETRY_SOURCE_H_
#define STEPIT_MODULAR_POLICY_LOCOMOTION_ODOMETRY_SOURCE_H_

#include <stepit/modular_policy/module.h>

namespace stepit {
namespace modular_policy {
class DummyOdometrySource : public Module {
 public:
  DummyOdometrySource(const ModularPolicySpec &, const ModuleSpec &module_spec);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &, FieldMap &context) override;

 protected:
  FieldId base_global_pos_id_{};
  FieldId base_global_ori_id_{};

  bool initialized_{false};
  Quatf world_frame_{};
};
}  // namespace modular_policy
}  // namespace stepit

#endif  // STEPIT_MODULAR_POLICY_LOCOMOTION_ODOMETRY_SOURCE_H_

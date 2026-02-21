#ifndef STEPIT_NEURO_POLICY_ODOMETRY_SOURCE_H_
#define STEPIT_NEURO_POLICY_ODOMETRY_SOURCE_H_

#include <stepit/policy_neuro/field.h>

namespace stepit {
namespace neuro_policy {
class DummyOdometrySource : public Module {
 public:
  DummyOdometrySource(const NeuroPolicySpec &, const std::string &name);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &, FieldMap &context) override;

 protected:
  FieldId base_global_pos_id_{};
  FieldId base_global_ori_id_{};

  bool initialized_{false};
  Quatf world_frame_{};
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_ODOMETRY_SOURCE_H_

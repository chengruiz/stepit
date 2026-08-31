#ifndef STEPIT_MODULAR_POLICY_FORWARD_KINEMATICS_H_
#define STEPIT_MODULAR_POLICY_FORWARD_KINEMATICS_H_

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <stepit/modular_policy/module.h>

namespace stepit {
namespace modular_policy {
class ForwardKinematics : public Module {
 public:
  ForwardKinematics(const ModularPolicySpec &policy_spec, const ModuleSpec &module_spec);

  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &, FieldMap &context) override;

 private:
  std::string urdf_path_;
  pinocchio::Model model_;
  pinocchio::Data data_;
  std::vector<std::string> body_names_;
  std::vector<pinocchio::FrameIndex> body_indices_;
  pinocchio::FrameIndex anchor_index_{};
  std::vector<pinocchio::JointIndex> joint_indices_;

  FieldId anchor_global_pos_id_{};
  FieldId anchor_global_ori_id_{};
  FieldId whole_body_local_pos_id_{};
  FieldId whole_body_local_ori_id_{};
  FieldId whole_body_global_pos_id_{};
  FieldId whole_body_global_ori_id_{};
};
}  // namespace modular_policy
}  // namespace stepit

#endif  // STEPIT_MODULAR_POLICY_FORWARD_KINEMATICS_H_

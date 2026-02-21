#ifndef STEPIT_NEURO_POLICY_FORWARD_KINEMATICS_H_
#define STEPIT_NEURO_POLICY_FORWARD_KINEMATICS_H_

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <stepit/policy_neuro/field.h>

namespace stepit {
namespace neuro_policy {
class ForwardKinematics : public Module {
 public:
  ForwardKinematics(const PolicySpec &policy_spec, const std::string &home_dir);

  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;

 private:
  YAML::Node config_;
  std::string urdf_filename_;
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
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_FORWARD_KINEMATICS_H_

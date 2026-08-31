#ifndef STEPIT_MODULAR_POLICY_RELATIVE_POSE_SOURCE_H_
#define STEPIT_MODULAR_POLICY_RELATIVE_POSE_SOURCE_H_

#include <stepit/modular_policy/module.h>

namespace stepit {
namespace modular_policy {
class RelativeOriSource : public Module {
 public:
  RelativeOriSource(const ModularPolicySpec &, const ModuleSpec &module_spec);
  bool init() override;
  bool update(const LowState &, ControlRequests &, FieldMap &context) override;

 protected:
  std::string current_ori_name_;
  std::string target_ori_name_;
  FieldId current_ori_id_{};
  FieldId target_ori_id_{};
  FieldId relative_ori_id_{};
  FieldId relative_ori_6d_id_{};
  Rotation6dOrder rot6d_order_{Rotation6dOrder::kRowMajor};
  std::size_t num_frames_{};
};

class RelativePosSource : public Module {
 public:
  RelativePosSource(const ModularPolicySpec &, const ModuleSpec &module_spec);
  bool init() override;
  bool update(const LowState &, ControlRequests &, FieldMap &context) override;

 protected:
  std::string current_pos_name_;
  std::string current_ori_name_;
  std::string target_pos_name_;
  FieldId current_pos_id_{};
  FieldId target_pos_id_{};
  FieldId current_ori_id_{};
  FieldId relative_pos_id_{};
  std::size_t num_frames_{};
};

class MotionAlignment : public Module {
 public:
  MotionAlignment(const ModularPolicySpec &, const ModuleSpec &module_spec);
  bool init() override;
  bool reset() override;
  bool update(const LowState &, ControlRequests &, FieldMap &context) override;

 protected:
  std::string current_pos_name_;
  std::string current_ori_name_;
  std::string target_pos_name_;
  std::string target_ori_name_;
  std::string alignment_trigger_name_{"motion_alignment_reset_event"};
  FieldId current_pos_id_{};
  FieldId current_ori_id_{};
  FieldId motion_restart_event_id_{};
  FieldId alignment_trigger_id_{kInvalidFieldId};
  FieldId target_pos_id_{};
  FieldId target_ori_id_{};
  FieldId aligned_target_pos_id_{};
  FieldId aligned_target_ori_id_{};
  std::size_t num_frames_{};
  int reference_index_{};
  std::size_t resolved_reference_index_{};

  Quatf world_to_init_yaw_;
  Vec3f world_to_init_pos_;
};
}  // namespace modular_policy
}  // namespace stepit

#endif  // STEPIT_MODULAR_POLICY_RELATIVE_POSE_SOURCE_H_

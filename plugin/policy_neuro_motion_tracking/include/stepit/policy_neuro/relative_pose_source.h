#ifndef STEPIT_NEURO_POLICY_RELATIVE_POSE_SOURCE_H_
#define STEPIT_NEURO_POLICY_RELATIVE_POSE_SOURCE_H_

#include <stepit/policy_neuro/module.h>

namespace stepit {
namespace neuro_policy {
enum class Rotation6dOrder { kRowMajor, kColumnMajor };

class RelativeOriSource : public Module {
 public:
  RelativeOriSource(const NeuroPolicySpec &, const ModuleSpec &module_spec);
  void init() override;
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
  RelativePosSource(const NeuroPolicySpec &, const ModuleSpec &module_spec);
  void init() override;
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
  MotionAlignment(const NeuroPolicySpec &, const ModuleSpec &module_spec);
  void init() override;
  bool reset() override;
  bool update(const LowState &, ControlRequests &, FieldMap &context) override;

 protected:
  std::string current_pos_name_;
  std::string current_ori_name_;
  std::string target_pos_name_;
  std::string target_ori_name_;
  FieldId current_pos_id_{};
  FieldId current_ori_id_{};
  FieldId motion_frame_index_id_{};
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
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_RELATIVE_POSE_SOURCE_H_

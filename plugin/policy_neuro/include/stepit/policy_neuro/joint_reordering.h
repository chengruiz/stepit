#ifndef STEPIT_NEURO_POLICY_JOINT_REORDERING_H_
#define STEPIT_NEURO_POLICY_JOINT_REORDERING_H_

#include <stepit/policy_neuro/field.h>

namespace stepit {
namespace neuro_policy {
class JointReordering : public Module {
 public:
  JointReordering(const NeuroPolicySpec &, const std::string &name);
  void initFieldProperties() override;
  bool reset() override { return true; }
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;

 private:
  ArrXf reorder(const ArrXf &in);

  YAML::Node config_;
  FieldId joint_pos_id_, joint_vel_id_;
  std::vector<std::size_t> joint_order_;
  std::vector<bool> joint_reversed_;
};

class ActionReordering : public Module {
 public:
  ActionReordering(const NeuroPolicySpec &, const std::string &name);
  void initFieldProperties() override;
  bool reset() override { return true; }
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;

 private:
  ArrXf reorder(const ArrXf &in);

  YAML::Node config_;
  FieldId action_id_;
  std::vector<std::size_t> joint_order_;
  std::vector<bool> joint_reversed_;
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_JOINT_REORDERING_H_

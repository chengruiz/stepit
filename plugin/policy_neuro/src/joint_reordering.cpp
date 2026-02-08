#include <algorithm>

#include <stepit/policy_neuro/joint_reordering.h>

namespace stepit {
namespace neuro_policy {
JointReordering::JointReordering(const PolicySpec &, const std::string &home_dir) {
  auto cfg = yml::loadFile(home_dir + "/joint_reordering.yml");
  yml::setTo(cfg, "order", joint_order_);
  yml::setIf(cfg, "reversed", joint_reversed_);

  auto sorted = joint_order_;
  std::sort(sorted.begin(), sorted.end());
  for (std::size_t i{}; i < sorted.size(); ++i) {
    STEPIT_ASSERT_EQ(sorted[i], i, "Joint order continuity check failed.");
  }
  if (joint_reversed_.empty()) {
    joint_reversed_.resize(joint_order_.size(), false);
  } else {
    STEPIT_ASSERT_EQ(joint_order_.size(), joint_reversed_.size(), "Sizes of 'order' and 'reversed' mismatch.");
  }

  joint_pos_id_ = registerRequirement("joint_pos");
  joint_vel_id_ = registerRequirement("joint_vel");
}

void JointReordering::initFieldProperties() {
  STEPIT_ASSERT_EQ(joint_order_.size(), getFieldSize(joint_pos_id_),
                   "Joint order size mismatch with joint_pos field size.");
  STEPIT_ASSERT_EQ(joint_order_.size(), getFieldSize(joint_vel_id_),
                   "Joint order size mismatch with joint_vel field size.");
}

ArrXf JointReordering::reorder(const ArrXf &in) {
  ArrXf out(in.size());
  for (Eigen::Index i{}; i < in.size(); ++i) {
    out[i] = in[static_cast<Eigen::Index>(joint_order_[i])] * (joint_reversed_[i] ? -1.0F : 1.0F);
  }
  return out;
}

bool JointReordering::update(const LowState &, ControlRequests &, FieldMap &result) {
  result[joint_pos_id_] = reorder(result.at(joint_pos_id_));
  result[joint_vel_id_] = reorder(result.at(joint_vel_id_));
  return true;
}

ActionReordering::ActionReordering(const PolicySpec &, const std::string &home_dir) {
  auto cfg = yml::loadFile(home_dir + "/joint_order.yml");
  yml::setTo(cfg, "order", joint_order_);
  yml::setIf(cfg, "reversed", joint_reversed_);

  auto sorted = joint_order_;
  std::sort(sorted.begin(), sorted.end());
  for (std::size_t i{}; i < sorted.size(); ++i) {
    STEPIT_ASSERT_EQ(sorted[i], i, "Joint order continuity check failed.");
  }
  if (joint_reversed_.empty()) {
    joint_reversed_.resize(joint_order_.size(), false);
  } else {
    STEPIT_ASSERT_EQ(joint_order_.size(), joint_reversed_.size(), "Sizes of 'order' and 'reversed' mismatch.");
  }

  action_id_ = registerRequirement("action");
}

void ActionReordering::initFieldProperties() {
  STEPIT_ASSERT_EQ(joint_order_.size(), getFieldSize(action_id_), "Joint order size mismatch with action field size.");
}

ArrXf ActionReordering::reorder(const ArrXf &in) {
  // The reverse of JointReordering::reorder
  ArrXf out(in.size());
  for (Eigen::Index i{}; i < in.size(); ++i) {
    out[static_cast<Eigen::Index>(joint_order_[i])] = in[i] * (joint_reversed_[i] ? -1.0F : 1.0F);
  }
  return out;
}

bool ActionReordering::update(const LowState &, ControlRequests &, FieldMap &result) {
  result[action_id_] = reorder(result.at(action_id_));
  return true;
}

STEPIT_REGISTER_MODULE(joint_reordering, kDefPriority, Module::make<JointReordering>);
STEPIT_REGISTER_MODULE(action_reordering, kDefPriority, Module::make<ActionReordering>);
}  // namespace neuro_policy
}  // namespace stepit

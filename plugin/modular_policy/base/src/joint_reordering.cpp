#include <algorithm>

#include <stepit/modular_policy/joint_reordering.h>

namespace stepit {
namespace modular_policy {
JointReordering::JointReordering(const ModularPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : Module(policy_spec, ModuleSpec(module_spec, "joint_reordering/measurements")) {
  yml::Node order_node    = config_["order"];
  yml::Node reversed_node = config_["reversed"];

  if (order_node.hasValue()) {
    order_node.assertSequence(policy_spec.dof);
    order_node.to(joint_order_);

    auto sorted = joint_order_;
    std::sort(sorted.begin(), sorted.end());
    for (std::size_t i{}; i < sorted.size(); ++i) {
      order_node.require(sorted[i] == i, "Joint order continuity check failed.");
    }
  } else {
    joint_order_.resize(policy_spec.dof);
    std::iota(joint_order_.begin(), joint_order_.end(), 0);
  }

  if (reversed_node.hasValue()) {
    reversed_node.assertSequence(policy_spec.dof);
    reversed_node.to(joint_reversed_);
  } else {
    joint_reversed_.resize(policy_spec.dof, false);
  }

  joint_pos_id_ = registerRequirement("joint_pos", DataType::kFloat32);
  joint_vel_id_ = registerRequirement("joint_vel", DataType::kFloat32);
}

bool JointReordering::init() {
  STEPIT_ASSERT_EQ(joint_order_.size(), getFieldSize(joint_pos_id_),
                   "Joint order size mismatch with joint_pos field size.");
  STEPIT_ASSERT_EQ(joint_order_.size(), getFieldSize(joint_vel_id_),
                   "Joint order size mismatch with joint_vel field size.");
  return true;
}

ArrXf JointReordering::reorder(Eigen::Ref<const ArrXf> in) {
  ArrXf out(in.size());
  for (Eigen::Index i{}; i < in.size(); ++i) {
    out[i] = in[static_cast<Eigen::Index>(joint_order_[i])] * (joint_reversed_[i] ? -1.0F : 1.0F);
  }
  return out;
}

bool JointReordering::update(const LowState &, ControlRequests &, FieldMap &context) {
  context[joint_pos_id_] = reorder(context.at(joint_pos_id_).view<float>());
  context[joint_vel_id_] = reorder(context.at(joint_vel_id_).view<float>());
  return true;
}

ActionReordering::ActionReordering(const ModularPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : Module(policy_spec, ModuleSpec(module_spec, "joint_reordering/action")) {
  yml::Node order_node    = config_["order"];
  yml::Node reversed_node = config_["reversed"];

  if (order_node.hasValue()) {
    order_node.assertSequence(policy_spec.dof);
    order_node.to(joint_order_);

    auto sorted = joint_order_;
    std::sort(sorted.begin(), sorted.end());
    for (std::size_t i{}; i < sorted.size(); ++i) {
      order_node.require(sorted[i] == i, "Joint order continuity check failed.");
    }
  } else {
    joint_order_.resize(policy_spec.dof);
    std::iota(joint_order_.begin(), joint_order_.end(), 0);
  }

  if (reversed_node.hasValue()) {
    reversed_node.assertSequence(policy_spec.dof);
    reversed_node.to(joint_reversed_);
  } else {
    joint_reversed_.resize(policy_spec.dof, false);
  }

  action_id_ = registerRequirement("action", DataType::kFloat32);
}

bool ActionReordering::init() {
  STEPIT_ASSERT_EQ(joint_order_.size(), getFieldSize(action_id_), "Joint order size mismatch with action field size.");
  return true;
}

ArrXf ActionReordering::reorder(Eigen::Ref<const ArrXf> in) {
  // The reverse of JointReordering::reorder
  ArrXf out(in.size());
  for (Eigen::Index i{}; i < in.size(); ++i) {
    out[static_cast<Eigen::Index>(joint_order_[i])] = in[i] * (joint_reversed_[i] ? -1.0F : 1.0F);
  }
  return out;
}

bool ActionReordering::update(const LowState &, ControlRequests &, FieldMap &context) {
  context[action_id_] = reorder(context.at(action_id_).view<float>());
  return true;
}

STEPIT_REGISTER_MODULE(joint_reordering, kDefPriority, Module::make<JointReordering>);
STEPIT_REGISTER_MODULE(action_reordering, kDefPriority, Module::make<ActionReordering>);
}  // namespace modular_policy
}  // namespace stepit

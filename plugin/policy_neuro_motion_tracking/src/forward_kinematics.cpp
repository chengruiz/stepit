#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <stepit/policy_neuro/forward_kinematics.h>

namespace stepit {
namespace neuro_policy {
ForwardKinematics::ForwardKinematics(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : Module(policy_spec, ModuleSpec(module_spec, "forward_kinematics")) {
  // Load URDF path and build Pinocchio model
  std::string urdf_path_key = config_.getDefinedKey({"urdf_filename", "urdf_path"});
  auto urdf_path_node       = config_[urdf_path_key];
  urdf_path_                = urdf_path_node.as<std::string>("robot.urdf");
  urdf_path_node.throwIf(urdf_path_.empty(), "'urdf_path' must not be empty");
  urdf_path_ = urdf_path_[0] == '/' ? urdf_path_ : joinPaths(policy_spec.home_dir, urdf_path_);
  pinocchio::urdf::buildModel(urdf_path_, pinocchio::JointModelFreeFlyer(), model_);
  data_ = pinocchio::Data(model_);

  // Load selected body names from config or use all body links from URDF by default
  if (config_["body_names"].hasValue()) {
    config_["body_names"].assertNonEmptySequence();
    config_["body_names"].to(body_names_);
  } else {
    for (const auto &frame : model_.frames) {
      if (frame.type == pinocchio::BODY) {
        body_names_.push_back(frame.name);
      }
    }
    STEPIT_ASSERT(not body_names_.empty(), "No body link found from URDF '{}'.", urdf_path_);
  }

  // Load anchor body name and get frame indices for the anchor and selected bodies
  auto anchor_body = config_["anchor_body"].as<std::string>(body_names_.front());
  anchor_index_    = model_.getFrameId(anchor_body);
  STEPIT_ASSERT(anchor_index_ < static_cast<pinocchio::FrameIndex>(model_.nframes),
                "Anchor frame '{}' is not found in urdf '{}'.", anchor_body, urdf_path_);
  STEPIT_DBUGNT("Forward kinematics body names:");
  for (const auto &name : body_names_) {
    STEPIT_DBUGNT("- {}", name);
    auto frame_idx = model_.getFrameId(name);
    STEPIT_ASSERT(frame_idx < static_cast<pinocchio::FrameIndex>(model_.nframes),
                  "Frame '{}' is not found in urdf '{}'.", name, urdf_path_);
    body_indices_.push_back(frame_idx);
  }

  // Get joint indices for the robot DoF specified in policy spec
  joint_indices_.reserve(policy_spec.joint_names.size());
  for (const auto &joint_name : policy_spec.joint_names) {
    auto joint_id = model_.getJointId(joint_name);
    STEPIT_ASSERT(joint_id < static_cast<pinocchio::JointIndex>(model_.njoints),
                  "Joint '{}' is not found in urdf '{}'.", joint_name, urdf_path_);
    int joint_nq = model_.nqs[joint_id], joint_nv = model_.nvs[joint_id];
    STEPIT_ASSERT(joint_nq == 1, "Joint '{}' has nq={}, only nq==1 is supported.", joint_name, joint_nq);
    STEPIT_ASSERT(joint_nv == 1, "Joint '{}' has nv={}, only nv==1 is supported.", joint_name, joint_nv);
    joint_indices_.push_back(joint_id);
  }
  STEPIT_ASSERT(policy_spec.dof == joint_indices_.size(),
                "Robot DoF ({}) does not match the number of joints {} specified in urdf '{}'.", policy_spec.dof,
                joint_indices_.size(), urdf_path_);

  // Register requirements for the anchor pose and provisions for the whole body poses
  auto anchor_global_pos_field = config_["anchor_global_pos_field"].as<std::string>("base_global_pos");
  anchor_global_pos_id_        = registerRequirement(anchor_global_pos_field, DataType::kFloat32, 3);
  auto anchor_global_ori_field = config_["anchor_global_ori_field"].as<std::string>("base_global_ori");
  anchor_global_ori_id_        = registerRequirement(anchor_global_ori_field, DataType::kFloat32, 4);

  const auto num_bodies     = body_names_.size();
  whole_body_local_pos_id_  = registerProvision("whole_body_local_pos", DataType::kFloat32, 3 * num_bodies);
  whole_body_local_ori_id_  = registerProvision("whole_body_local_ori", DataType::kFloat32, 4 * num_bodies);
  whole_body_global_pos_id_ = registerProvision("whole_body_global_pos", DataType::kFloat32, 3 * num_bodies);
  whole_body_global_ori_id_ = registerProvision("whole_body_global_ori", DataType::kFloat32, 4 * num_bodies);
}

bool ForwardKinematics::reset() { return true; }

bool ForwardKinematics::update(const LowState &low_state, ControlRequests &, FieldMap &context) {
  pinocchio::Model::ConfigVectorType q  = pinocchio::neutral(model_);
  pinocchio::Model::TangentVectorType v = pinocchio::Model::TangentVectorType::Zero(model_.nv);

  for (std::size_t i{}; i < joint_indices_.size(); ++i) {
    const auto joint_id        = joint_indices_[i];
    q[model_.idx_qs[joint_id]] = low_state.motor_state[i].q;
    v[model_.idx_vs[joint_id]] = low_state.motor_state[i].dq;
  }

  pinocchio::forwardKinematics(model_, data_, q, v);
  pinocchio::updateFramePlacements(model_, data_);

  ArrXf whole_body_local_pos(3 * body_indices_.size());
  ArrXf whole_body_local_ori(4 * body_indices_.size());

  const auto &anchor_pose = data_.oMf[anchor_index_];
  for (std::size_t i = 0; i < body_indices_.size(); ++i) {
    const auto body_pose_local             = anchor_pose.actInv(data_.oMf[body_indices_[i]]);
    whole_body_local_pos.segment(3 * i, 3) = body_pose_local.translation().cast<float>();

    Quatf quat                             = Quatf::fromMatrix(body_pose_local.rotation().cast<float>());
    whole_body_local_ori.segment(4 * i, 4) = quat.coeffs();
  }

  ArrXf whole_body_global_pos = whole_body_local_pos;
  ArrXf whole_body_global_ori = whole_body_local_ori;
  Quatf anchor_global_ori(context.at(anchor_global_ori_id_).get<float>());
  for (std::size_t i{}; i < body_indices_.size(); ++i) {
    Quatf local_ori(whole_body_local_ori.segment(4 * i, 4));
    Quatf global_ori = anchor_global_ori * local_ori;

    whole_body_global_ori.segment(4 * i, 4) = global_ori.coeffs();
  }

  Vec3f anchor_global_pos = context.at(anchor_global_pos_id_).get<float>();
  for (std::size_t i{}; i < body_indices_.size(); ++i) {
    Vec3f body_local_pos                    = whole_body_local_pos.segment(3 * i, 3);
    whole_body_global_pos.segment(3 * i, 3) = anchor_global_ori * body_local_pos + anchor_global_pos;
  }

  context[whole_body_local_pos_id_]  = whole_body_local_pos;
  context[whole_body_local_ori_id_]  = whole_body_local_ori;
  context[whole_body_global_pos_id_] = whole_body_global_pos;
  context[whole_body_global_ori_id_] = whole_body_global_ori;
  return true;
}

STEPIT_REGISTER_MODULE(forward_kinematics, kDefPriority, Module::make<ForwardKinematics>);
STEPIT_REGISTER_FIELD_SOURCE(whole_body_local_pos, kDefPriority, Module::make<ForwardKinematics>);
STEPIT_REGISTER_FIELD_SOURCE(whole_body_local_ori, kDefPriority, Module::make<ForwardKinematics>);
STEPIT_REGISTER_FIELD_SOURCE(whole_body_global_pos, kDefPriority, Module::make<ForwardKinematics>);
STEPIT_REGISTER_FIELD_SOURCE(whole_body_global_ori, kDefPriority, Module::make<ForwardKinematics>);
}  // namespace neuro_policy
}  // namespace stepit

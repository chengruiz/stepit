#include <stepit/policy_neuro/relative_pose_source.h>

namespace stepit {
namespace neuro_policy {
RelativeOriSource::RelativeOriSource(const NeuroPolicySpec &policy_spec, const std::string &name)
    : Module(nonEmptyOr(name, "relative_pose/ori")), config_(loadConfigIf(policy_spec)) {
  current_ori_name_ = yml::readIf<std::string>(config_, "current_ori_name", "base_global_ori");
  target_ori_name_  = yml::readIf<std::string>(config_, "target_ori_name", "base_target_ori");
  auto rot6d_order  = yml::readIf<std::string>(config_, "rotation_6d_order", "row_major");
  if (rot6d_order == "row_major") {
    rot6d_order_ = Rotation6dOrder::kRowMajor;
  } else if (rot6d_order == "column_major") {
    rot6d_order_ = Rotation6dOrder::kColumnMajor;
  } else {
    STEPIT_THROW("Unsupported 'rotation_6d_order': '{}'. Expected 'column_major' or 'row_major'.", rot6d_order);
  }

  current_ori_id_     = registerRequirement(current_ori_name_, 4);
  target_ori_id_      = registerRequirement(target_ori_name_, 4);
  relative_ori_id_    = registerProvision("relative_ori", 4);
  relative_ori_6d_id_ = registerProvision("relative_ori_6d", 6);
}

bool RelativeOriSource::update(const LowState &, ControlRequests &, FieldMap &context) {
  Quatf current_ori(context.at(current_ori_id_));
  Quatf target_ori(context.at(target_ori_id_));

  Quatf relative_ori        = current_ori.inverse() * target_ori;
  context[relative_ori_id_] = relative_ori.coeffs();

  Mat3f rotation_matrix = relative_ori.matrix();
  Vec6f relative_ori_6d;
  if (rot6d_order_ == Rotation6dOrder::kColumnMajor) {
    // clang-format off
    relative_ori_6d <<
        rotation_matrix(0, 0), rotation_matrix(1, 0), rotation_matrix(2, 0),
        rotation_matrix(0, 1), rotation_matrix(1, 1), rotation_matrix(2, 1);
    // clang-format on
  } else {
    // clang-format off
    relative_ori_6d <<
        rotation_matrix(0, 0), rotation_matrix(0, 1),
        rotation_matrix(1, 0), rotation_matrix(1, 1),
        rotation_matrix(2, 0), rotation_matrix(2, 1);
    // clang-format on
  }
  context[relative_ori_6d_id_] = relative_ori_6d;
  return true;
}

RelativePosSource::RelativePosSource(const NeuroPolicySpec &policy_spec, const std::string &name)
    : Module(nonEmptyOr(name, "relative_pose/pos")), config_(loadConfigIf(policy_spec)) {
  current_pos_name_ = yml::readIf<std::string>(config_, "current_pos_name", "base_global_pos");
  current_ori_name_ = yml::readIf<std::string>(config_, "current_ori_name", "base_global_ori");
  target_pos_name_  = yml::readIf<std::string>(config_, "target_pos_name", "base_target_pos");

  current_pos_id_  = registerRequirement(current_pos_name_, 3);
  current_ori_id_  = registerRequirement(current_ori_name_, 4);
  target_pos_id_   = registerRequirement(target_pos_name_, 3);
  relative_pos_id_ = registerProvision("relative_pos", 3);
}

bool RelativePosSource::update(const LowState &, ControlRequests &, FieldMap &context) {
  Vec3f current_pos(context.at(current_pos_id_));
  Quatf current_ori(context.at(current_ori_id_));
  Vec3f target_pos(context.at(target_pos_id_));

  Vec3f relative_pos        = current_ori.inverse() * (target_pos - current_pos);
  context[relative_pos_id_] = relative_pos;
  return true;
}

MotionAlignment::MotionAlignment(const NeuroPolicySpec &policy_spec, const std::string &name)
    : Module(nonEmptyOr(name, "motion_alignment")), config_(loadConfigIf(policy_spec)) {
  initialized_ = false;
  world_to_init_yaw_.setIdentity();
  world_to_init_pos_ = Vec3f::Zero();

  current_pos_name_ = yml::readIf<std::string>(config_, "current_pos_name", "base_global_pos");
  current_ori_name_ = yml::readIf<std::string>(config_, "current_ori_name", "base_global_ori");
  target_pos_name_  = yml::readIf<std::string>(config_, "target_pos_name", "base_target_pos");
  target_ori_name_  = yml::readIf<std::string>(config_, "target_ori_name", "base_target_ori");

  current_ori_id_        = registerRequirement(current_ori_name_, 4);
  target_ori_id_         = registerRequirement(target_ori_name_, 4);
  aligned_target_ori_id_ = registerProvision("aligned_target_ori", 4);

  if (not target_pos_name_.empty()) {
    current_pos_id_ = registerRequirement(current_pos_name_, 3);
    target_pos_id_  = registerRequirement(target_pos_name_, 3);
  } else {
    current_pos_id_ = kInvalidFieldId;
    target_pos_id_  = kInvalidFieldId;
  }
  aligned_target_pos_id_ = registerProvision("aligned_target_pos", 3);
}

bool MotionAlignment::reset() {
  initialized_ = false;
  world_to_init_yaw_.setIdentity();
  world_to_init_pos_ = Vec3f::Zero();
  return true;
}

bool MotionAlignment::update(const LowState &, ControlRequests &, FieldMap &context) {
  Quatf current_ori(context.at(current_ori_id_));
  Quatf target_ori(context.at(target_ori_id_));
  Vec3f current_pos = current_pos_id_ == kInvalidFieldId ? Vec3f::Zero() : context.at(current_pos_id_);
  Vec3f target_pos  = target_pos_id_ == kInvalidFieldId ? Vec3f::Zero() : context.at(target_pos_id_);

  if (not initialized_) {
    Quatf current_yaw  = Quatf::fromYaw(current_ori.eulerAngles().z());
    Quatf target_yaw   = Quatf::fromYaw(target_ori.eulerAngles().z());
    world_to_init_yaw_ = current_yaw * target_yaw.inverse();
    world_to_init_pos_ = current_pos - world_to_init_yaw_ * target_pos;
    initialized_       = true;
  }

  Quatf aligned_target_ori        = world_to_init_yaw_ * target_ori;
  Vec3f aligned_target_pos        = world_to_init_yaw_ * target_pos + world_to_init_pos_;
  context[aligned_target_ori_id_] = aligned_target_ori.coeffs();
  context[aligned_target_pos_id_] = aligned_target_pos;
  return true;
}

STEPIT_REGISTER_MODULE(relative_ori, kDefPriority, Module::make<RelativeOriSource>);
STEPIT_REGISTER_MODULE(relative_pos, kDefPriority, Module::make<RelativePosSource>);
STEPIT_REGISTER_MODULE(motion_alignment, kDefPriority, Module::make<MotionAlignment>);
STEPIT_REGISTER_FIELD_SOURCE(relative_ori, kDefPriority, Module::make<RelativeOriSource>);
STEPIT_REGISTER_FIELD_SOURCE(relative_ori_6d, kDefPriority, Module::make<RelativeOriSource>);
STEPIT_REGISTER_FIELD_SOURCE(relative_pos, kDefPriority, Module::make<RelativePosSource>);
STEPIT_REGISTER_FIELD_SOURCE(aligned_target_pos, kDefPriority, Module::make<MotionAlignment>);
STEPIT_REGISTER_FIELD_SOURCE(aligned_target_ori, kDefPriority, Module::make<MotionAlignment>);
}  // namespace neuro_policy
}  // namespace stepit

#include <cstddef>
#include <stepit/policy_neuro/relative_pose_source.h>

namespace stepit {
namespace neuro_policy {
RelativeOriSource::RelativeOriSource(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : Module(policy_spec, ModuleSpec(module_spec, "relative_pose/ori")) {
  current_ori_name_ = config_["current_ori_name"].as<std::string>("base_global_ori");
  target_ori_name_  = config_["target_ori_name"].as<std::string>("base_target_ori");
  auto rot6d_order  = config_["rotation_6d_order"].as<std::string>("row_major");
  if (rot6d_order == "row_major") {
    rot6d_order_ = Rotation6dOrder::kRowMajor;
  } else if (rot6d_order == "column_major") {
    rot6d_order_ = Rotation6dOrder::kColumnMajor;
  } else {
    STEPIT_THROW("Unsupported 'rotation_6d_order': '{}'. Expected 'column_major' or 'row_major'.", rot6d_order);
  }

  current_ori_id_     = registerRequirement(current_ori_name_, 4);
  target_ori_id_      = registerRequirement(target_ori_name_);
  relative_ori_id_    = registerProvision("relative_ori", 0);
  relative_ori_6d_id_ = registerProvision("relative_ori_6d", 0);
}

void RelativeOriSource::init() {
  auto target_ori_size = getFieldSize(target_ori_id_);
  STEPIT_ASSERT(target_ori_size > 0 and target_ori_size % 4 == 0, "Field '{}' must have size 4 * N, but got {}.",
                target_ori_name_, target_ori_size);
  num_frames_ = target_ori_size / 4;
  setFieldSize(relative_ori_id_, static_cast<FieldSize>(4 * num_frames_));
  setFieldSize(relative_ori_6d_id_, static_cast<FieldSize>(6 * num_frames_));
}

bool RelativeOriSource::update(const LowState &, ControlRequests &, FieldMap &context) {
  Quatf current_ori(context.at(current_ori_id_));
  const auto &target_ori_stack = context.at(target_ori_id_);

  ArrXf relative_ori(4 * num_frames_);
  ArrXf relative_ori_6d(6 * num_frames_);
  for (std::size_t i{}; i < num_frames_; ++i) {
    Quatf target_ori(target_ori_stack.segment(4 * i, 4));

    Quatf relative_ori_i           = current_ori.inverse() * target_ori;
    relative_ori.segment(4 * i, 4) = relative_ori_i.coeffs();

    Mat3f relative_rot_i = relative_ori_i.matrix();
    if (rot6d_order_ == Rotation6dOrder::kColumnMajor) {
      relative_ori_6d.segment(6 * i, 6) << relative_rot_i(0, 0), relative_rot_i(1, 0), relative_rot_i(2, 0),
          relative_rot_i(0, 1), relative_rot_i(1, 1), relative_rot_i(2, 1);
    } else {
      relative_ori_6d.segment(6 * i, 6) << relative_rot_i(0, 0), relative_rot_i(0, 1), relative_rot_i(1, 0),
          relative_rot_i(1, 1), relative_rot_i(2, 0), relative_rot_i(2, 1);
    }
  }

  context[relative_ori_id_]    = relative_ori;
  context[relative_ori_6d_id_] = relative_ori_6d;
  return true;
}

RelativePosSource::RelativePosSource(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : Module(policy_spec, ModuleSpec(module_spec, "relative_pose/pos")) {
  current_pos_name_ = config_["current_pos_name"].as<std::string>("base_global_pos");
  current_ori_name_ = config_["current_ori_name"].as<std::string>("base_global_ori");
  target_pos_name_  = config_["target_pos_name"].as<std::string>("base_target_pos");

  current_pos_id_  = registerRequirement(current_pos_name_, 3);
  current_ori_id_  = registerRequirement(current_ori_name_, 4);
  target_pos_id_   = registerRequirement(target_pos_name_);
  relative_pos_id_ = registerProvision("relative_pos", 0);
}

void RelativePosSource::init() {
  auto target_pos_size = getFieldSize(target_pos_id_);
  STEPIT_ASSERT(target_pos_size > 0 and target_pos_size % 3 == 0, "Field '{}' must have size 3 * N, but got {}.",
                target_pos_name_, target_pos_size);
  num_frames_ = target_pos_size / 3;
  setFieldSize(relative_pos_id_, static_cast<FieldSize>(3 * num_frames_));
}

bool RelativePosSource::update(const LowState &, ControlRequests &, FieldMap &context) {
  Vec3f current_pos(context.at(current_pos_id_));
  Quatf current_ori(context.at(current_ori_id_));
  const auto &target_pos = context.at(target_pos_id_);

  ArrXf relative_pos(3 * num_frames_);
  for (std::size_t i{}; i < num_frames_; ++i) {
    Vec3f target_pos_i             = target_pos.segment(3 * i, 3);
    Vec3f relative_pos_i           = current_ori.inverse() * (target_pos_i - current_pos);
    relative_pos.segment(3 * i, 3) = relative_pos_i;
  }

  context[relative_pos_id_] = relative_pos;
  return true;
}

MotionAlignment::MotionAlignment(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : Module(policy_spec, ModuleSpec(module_spec, "motion_alignment")) {
  world_to_init_yaw_.setIdentity();
  world_to_init_pos_.setZero();
  reference_index_          = config_["reference_index"].as<int>(0);
  resolved_reference_index_ = 0;

  current_pos_name_ = config_["current_pos_name"].as<std::string>("base_global_pos");
  current_ori_name_ = config_["current_ori_name"].as<std::string>("base_global_ori");
  target_pos_name_  = config_["target_pos_name"].as<std::string>("base_target_pos");
  target_ori_name_  = config_["target_ori_name"].as<std::string>("base_target_ori");

  current_ori_id_        = registerRequirement(current_ori_name_, 4);
  target_ori_id_         = registerRequirement(target_ori_name_);
  motion_frame_index_id_ = registerRequirement("motion_frame_index", 1);
  aligned_target_ori_id_ = registerProvision("aligned_target_ori", 0);

  if (not target_pos_name_.empty()) {
    current_pos_id_ = registerRequirement(current_pos_name_, 3);
    target_pos_id_  = registerRequirement(target_pos_name_);
  } else {
    current_pos_id_ = kInvalidFieldId;
    target_pos_id_  = kInvalidFieldId;
  }
  aligned_target_pos_id_ = registerProvision("aligned_target_pos", 0);
}

void MotionAlignment::init() {
  auto target_ori_size = getFieldSize(target_ori_id_);
  STEPIT_ASSERT(target_ori_size > 0 and target_ori_size % 4 == 0, "Field '{}' must have size 4 * N, but got {}.",
                target_ori_name_, target_ori_size);
  num_frames_ = target_ori_size / 4;
  STEPIT_ASSERT(reference_index_ >= -static_cast<int>(num_frames_) and reference_index_ < static_cast<int>(num_frames_),
                "'reference_index'={} is out of range for {} frames. Expected [{}, {}).", reference_index_, num_frames_,
                -static_cast<int>(num_frames_), static_cast<int>(num_frames_));
  resolved_reference_index_ = reference_index_ >= 0 ? static_cast<std::size_t>(reference_index_)
                                                    : num_frames_ + reference_index_;
  setFieldSize(aligned_target_ori_id_, static_cast<FieldSize>(4 * num_frames_));

  if (target_pos_id_ != kInvalidFieldId) {
    auto target_pos_size = getFieldSize(target_pos_id_);
    STEPIT_ASSERT(target_pos_size > 0 and target_pos_size % 3 == 0, "Field '{}' must have size 3 * N, but got {}.",
                  target_pos_name_, target_pos_size);
    std::size_t target_pos_num_frames = target_pos_size / 3;
    STEPIT_ASSERT(target_pos_num_frames == num_frames_,
                  "Field '{}' must have {} frames to match '{}', but got {} frames.", target_pos_name_, num_frames_,
                  target_ori_name_, target_pos_num_frames);
  }
  setFieldSize(aligned_target_pos_id_, static_cast<FieldSize>(3 * num_frames_));
}

bool MotionAlignment::reset() {
  world_to_init_yaw_.setIdentity();
  world_to_init_pos_.setZero();
  return true;
}

bool MotionAlignment::update(const LowState &, ControlRequests &, FieldMap &context) {
  Quatf current_ori(context.at(current_ori_id_));
  const auto &target_ori = context.at(target_ori_id_);

  Vec3f current_pos = current_pos_id_ == kInvalidFieldId ? Vec3f::Zero() : context.at(current_pos_id_);
  ArrXf target_pos  = target_pos_id_ == kInvalidFieldId ? ArrXf::Zero(target_ori.size()) : context.at(target_pos_id_);

  if (static_cast<std::size_t>(context.at(motion_frame_index_id_)(0)) == 0) {
    Quatf reference_ori(target_ori.segment(4 * resolved_reference_index_, 4));
    Quatf current_yaw   = Quatf::fromYaw(current_ori.eulerAngles().z());
    Quatf reference_yaw = Quatf::fromYaw(reference_ori.eulerAngles().z());
    Vec3f reference_pos = target_pos.segment(3 * resolved_reference_index_, 3);
    world_to_init_yaw_  = current_yaw * reference_yaw.inverse();
    world_to_init_pos_  = current_pos - world_to_init_yaw_ * reference_pos;
  }

  ArrXf aligned_target_ori(4 * num_frames_);
  ArrXf aligned_target_pos(3 * num_frames_);
  for (std::size_t i{}; i < num_frames_; ++i) {
    Quatf target_ori_i(target_ori.segment(4 * i, 4));
    Quatf aligned_target_ori_i           = world_to_init_yaw_ * target_ori_i;
    aligned_target_ori.segment(4 * i, 4) = aligned_target_ori_i.coeffs();
    Vec3f target_pos_i                   = target_pos.segment(3 * i, 3).matrix();
    Vec3f aligned_target_pos_i           = world_to_init_yaw_ * target_pos_i + world_to_init_pos_;
    aligned_target_pos.segment(3 * i, 3) = aligned_target_pos_i;
  }
  context[aligned_target_ori_id_] = aligned_target_ori;
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

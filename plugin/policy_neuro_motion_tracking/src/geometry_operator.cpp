#include <stepit/policy_neuro/geometry_operator.h>

namespace stepit {
namespace field {
QuatRotateOperator::QuatRotateOperator(const yml::Node &config) {
  config.assertHasValue("source", "quaternion", "target");
  source_id_ = registerRequirement(config["source"].as<std::string>(), DataType::kFloat32);
  quat_id_   = registerRequirement(config["quaternion"].as<std::string>(), DataType::kFloat32);
  target_id_ = registerProvision(config["target"].as<std::string>(), DataType::kFloat32);
  inverse_   = config["inverse"].as<bool>(false);
}

void QuatRotateOperator::init() {
  if (buffer_.size() > 0) return;
  const auto source_size = getFieldSize(source_id_);
  const auto quat_size   = getFieldSize(quat_id_);
  STEPIT_ASSERT(source_size > 0 and source_size % 3 == 0, "Field '{}' must have size 3 * N, but got {}.",
                getFieldName(source_id_), source_size);
  STEPIT_ASSERT(quat_size > 0 and quat_size % 4 == 0, "Field '{}' must have size 4 * M, but got {}.",
                getFieldName(quat_id_), quat_size);

  source_size_ = source_size;
  num_vectors_ = source_size / 3;
  num_quats_   = quat_size / 4;
  STEPIT_ASSERT(num_quats_ == 1 or num_quats_ == num_vectors_,
                "Quaternion field '{}' must contain 1 quaternion or match vector count {}.", getFieldName(quat_id_),
                num_vectors_);

  setFieldSpec(target_id_, FieldSpec{DataType::kFloat32, source_size});
  buffer_.resize(source_size);
}

bool QuatRotateOperator::update(FieldMap &context) {
  const auto source = context.at(source_id_).view<float>();
  const auto quat   = context.at(quat_id_).view<float>();

  for (std::size_t i{}; i < num_vectors_; ++i) {
    const auto quat_offset = 4 * (num_quats_ == 1 ? 0 : i);
    Quatf q(quat.segment(static_cast<Eigen::Index>(quat_offset), 4));
    if (inverse_) q = q.inverse();
    buffer_.segment(static_cast<Eigen::Index>(3 * i),
                    3) = (q * source.segment(static_cast<Eigen::Index>(3 * i), 3).matrix()).array();
  }
  context[target_id_] = buffer_;
  return true;
}

QuatRotateBetweenOperator::QuatRotateBetweenOperator(const yml::Node &config) {
  config.assertHasValue("source", "from_quaternion", "to_quaternion", "target");
  source_id_    = registerRequirement(config["source"].as<std::string>(), DataType::kFloat32);
  from_quat_id_ = registerRequirement(config["from_quaternion"].as<std::string>(), DataType::kFloat32);
  to_quat_id_   = registerRequirement(config["to_quaternion"].as<std::string>(), DataType::kFloat32);
  target_id_    = registerProvision(config["target"].as<std::string>(), DataType::kFloat32);
}

void QuatRotateBetweenOperator::init() {
  if (buffer_.size() > 0) return;
  const auto source_size = getFieldSize(source_id_);
  const auto from_size   = getFieldSize(from_quat_id_);
  const auto to_size     = getFieldSize(to_quat_id_);
  STEPIT_ASSERT(source_size > 0 and source_size % 3 == 0, "Field '{}' must have size 3 * N, but got {}.",
                getFieldName(source_id_), source_size);
  STEPIT_ASSERT(from_size > 0 and from_size % 4 == 0, "Field '{}' must have size 4 * M, but got {}.",
                getFieldName(from_quat_id_), from_size);
  STEPIT_ASSERT(to_size > 0 and to_size % 4 == 0, "Field '{}' must have size 4 * M, but got {}.",
                getFieldName(to_quat_id_), to_size);

  source_size_    = source_size;
  num_vectors_    = source_size / 3;
  num_from_quats_ = from_size / 4;
  num_to_quats_   = to_size / 4;
  STEPIT_ASSERT(num_from_quats_ == 1 or num_from_quats_ == num_vectors_,
                "Quaternion field '{}' must contain 1 quaternion or match vector count {}.",
                getFieldName(from_quat_id_), num_vectors_);
  STEPIT_ASSERT(num_to_quats_ == 1 or num_to_quats_ == num_vectors_,
                "Quaternion field '{}' must contain 1 quaternion or match vector count {}.", getFieldName(to_quat_id_),
                num_vectors_);

  setFieldSpec(target_id_, FieldSpec{DataType::kFloat32, source_size});
  buffer_.resize(source_size);
}

bool QuatRotateBetweenOperator::update(FieldMap &context) {
  const auto source    = context.at(source_id_).view<float>();
  const auto from_quat = context.at(from_quat_id_).view<float>();
  const auto to_quat   = context.at(to_quat_id_).view<float>();

  for (std::size_t i{}; i < num_vectors_; ++i) {
    const auto from_offset = 4 * (num_from_quats_ == 1 ? 0 : i);
    const auto to_offset   = 4 * (num_to_quats_ == 1 ? 0 : i);
    Quatf q_from(from_quat.segment(static_cast<Eigen::Index>(from_offset), 4));
    Quatf q_to(to_quat.segment(static_cast<Eigen::Index>(to_offset), 4));
    const auto q_delta = q_to * q_from.inverse();
    buffer_.segment(static_cast<Eigen::Index>(3 * i),
                    3) = (q_delta * source.segment(static_cast<Eigen::Index>(3 * i), 3).matrix()).array();
  }
  context[target_id_] = buffer_;
  return true;
}

QuatInverseOperator::QuatInverseOperator(const yml::Node &config) {
  config.assertHasValue("source", "target");
  source_id_ = registerRequirement(config["source"].as<std::string>(), DataType::kFloat32);
  target_id_ = registerProvision(config["target"].as<std::string>(), DataType::kFloat32);
}

void QuatInverseOperator::init() {
  if (buffer_.size() > 0) return;
  const auto source_size = getFieldSize(source_id_);
  STEPIT_ASSERT(source_size > 0 and source_size % 4 == 0, "Field '{}' must have size 4 * N, but got {}.",
                getFieldName(source_id_), source_size);
  num_quats_ = source_size / 4;
  setFieldSpec(target_id_, FieldSpec{DataType::kFloat32, source_size});
  buffer_.resize(source_size);
}

bool QuatInverseOperator::update(FieldMap &context) {
  const auto source = context.at(source_id_).view<float>();
  for (std::size_t i{}; i < num_quats_; ++i) {
    Quatf q(source.segment(static_cast<Eigen::Index>(4 * i), 4));
    buffer_.segment(static_cast<Eigen::Index>(4 * i), 4) = q.inverse().coeffs();
  }
  context[target_id_] = buffer_;
  return true;
}

QuatToEulerOperator::QuatToEulerOperator(const yml::Node &config) {
  config.assertHasValue("source", "target");
  source_id_ = registerRequirement(config["source"].as<std::string>(), DataType::kFloat32);
  target_id_ = registerProvision(config["target"].as<std::string>(), DataType::kFloat32);
}

void QuatToEulerOperator::init() {
  if (buffer_.size() > 0) return;
  const auto source_size = getFieldSize(source_id_);
  STEPIT_ASSERT(source_size > 0 and source_size % 4 == 0, "Field '{}' must have size 4 * N, but got {}.",
                getFieldName(source_id_), source_size);
  num_quats_ = source_size / 4;
  setFieldSpec(target_id_, FieldSpec{DataType::kFloat32, 3 * num_quats_});
  buffer_.resize(getFieldSize(target_id_));
}

bool QuatToEulerOperator::update(FieldMap &context) {
  const auto source = context.at(source_id_).view<float>();
  for (std::size_t i{}; i < num_quats_; ++i) {
    Quatf q(source.segment(static_cast<Eigen::Index>(4 * i), 4));
    buffer_.segment(static_cast<Eigen::Index>(3 * i), 3) = q.eulerAngles().array();
  }
  context[target_id_] = buffer_;
  return true;
}

EulerToQuatOperator::EulerToQuatOperator(const yml::Node &config) {
  config.assertHasValue("source", "target");
  source_id_ = registerRequirement(config["source"].as<std::string>(), DataType::kFloat32);
  target_id_ = registerProvision(config["target"].as<std::string>(), DataType::kFloat32);
}

void EulerToQuatOperator::init() {
  if (buffer_.size() > 0) return;
  const auto source_size = getFieldSize(source_id_);
  STEPIT_ASSERT(source_size > 0 and source_size % 3 == 0, "Field '{}' must have size 3 * N, but got {}.",
                getFieldName(source_id_), source_size);
  num_rpys_ = source_size / 3;
  setFieldSpec(target_id_, FieldSpec{DataType::kFloat32, 4 * num_rpys_});
  buffer_.resize(getFieldSize(target_id_));
}

bool EulerToQuatOperator::update(FieldMap &context) {
  const auto source = context.at(source_id_).view<float>();
  for (std::size_t i{}; i < num_rpys_; ++i) {
    Quatf q = Quatf::fromEulerAngles(source.segment(static_cast<Eigen::Index>(3 * i), 3).matrix());
    buffer_.segment(static_cast<Eigen::Index>(4 * i), 4) = q.coeffs();
  }
  context[target_id_] = buffer_;
  return true;
}

QuatToAngleAxisOperator::QuatToAngleAxisOperator(const yml::Node &config) {
  config.assertHasValue("source", "target");
  source_id_ = registerRequirement(config["source"].as<std::string>(), DataType::kFloat32);
  target_id_ = registerProvision(config["target"].as<std::string>(), DataType::kFloat32);
}

void QuatToAngleAxisOperator::init() {
  if (buffer_.size() > 0) return;
  const auto source_size = getFieldSize(source_id_);
  STEPIT_ASSERT(source_size > 0 and source_size % 4 == 0, "Field '{}' must have size 4 * N, but got {}.",
                getFieldName(source_id_), source_size);
  num_quats_ = source_size / 4;
  setFieldSpec(target_id_, FieldSpec{DataType::kFloat32, 3 * num_quats_});
  buffer_.resize(getFieldSize(target_id_));
}

bool QuatToAngleAxisOperator::update(FieldMap &context) {
  const auto source = context.at(source_id_).view<float>();
  for (std::size_t i{}; i < num_quats_; ++i) {
    Quatf q(source.segment(static_cast<Eigen::Index>(4 * i), 4));
    buffer_.segment(static_cast<Eigen::Index>(3 * i), 3) = q.rotationVector();
  }
  context[target_id_] = buffer_;
  return true;
}

QuatToRotation6dOperator::QuatToRotation6dOperator(const yml::Node &config) {
  config.assertHasValue("source", "target");
  source_id_ = registerRequirement(config["source"].as<std::string>(), DataType::kFloat32);
  target_id_ = registerProvision(config["target"].as<std::string>(), DataType::kFloat32);
  order_     = config["rotation_6d_order"].as(Rotation6dOrder::kRowMajor);
}

void QuatToRotation6dOperator::init() {
  if (buffer_.size() > 0) return;
  const auto source_size = getFieldSize(source_id_);
  STEPIT_ASSERT(source_size > 0 and source_size % 4 == 0, "Field '{}' must have size 4 * N, but got {}.",
                getFieldName(source_id_), source_size);
  num_quats_ = source_size / 4;
  setFieldSpec(target_id_, FieldSpec{DataType::kFloat32, 6 * num_quats_});
  buffer_.resize(getFieldSize(target_id_));
}

bool QuatToRotation6dOperator::update(FieldMap &context) {
  const auto source = context.at(source_id_).view<float>();
  for (std::size_t i{}; i < num_quats_; ++i) {
    Quatf q(source.segment(static_cast<Eigen::Index>(4 * i), 4));
    buffer_.segment(static_cast<Eigen::Index>(6 * i), 6) = q.rotation6d(order_);
  }
  context[target_id_] = buffer_;
  return true;
}

STEPIT_REGISTER_FIELD_OPERATOR(quat_rotate, kDefPriority, Operator::make<QuatRotateOperator>);
STEPIT_REGISTER_FIELD_OPERATOR(quat_rotate_between, kDefPriority, Operator::make<QuatRotateBetweenOperator>);
STEPIT_REGISTER_FIELD_OPERATOR(quat_inverse, kDefPriority, Operator::make<QuatInverseOperator>);
STEPIT_REGISTER_FIELD_OPERATOR(quat_to_euler, kDefPriority, Operator::make<QuatToEulerOperator>);
STEPIT_REGISTER_FIELD_OPERATOR(euler_to_quat, kDefPriority, Operator::make<EulerToQuatOperator>);
STEPIT_REGISTER_FIELD_OPERATOR(quat_to_angle_axis, kDefPriority, Operator::make<QuatToAngleAxisOperator>);
STEPIT_REGISTER_FIELD_OPERATOR(quat_to_rotation6d, kDefPriority, Operator::make<QuatToRotation6dOperator>);
}  // namespace field
}  // namespace stepit

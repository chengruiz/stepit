#include <algorithm>
#include <utility>

#include <stepit/ros2/node.h>
#include <stepit/ros2/publisher.h>

namespace stepit {
template <typename Message, typename Scalar>
Ros2Publisher::ArrayPublisher::TypedPublisher<Message, Scalar>::TypedPublisher(
    const rclcpp::Node::SharedPtr &node, const std::string &name)
    : publisher_(node->create_publisher<Message>(name, 1)) {}

template <typename Message, typename Scalar>
void Ros2Publisher::ArrayPublisher::TypedPublisher<Message, Scalar>::publish(const void *data, std::size_t size) {
  Message msg;
  msg.data.resize(size);
  if (size > 0) std::copy_n(static_cast<const Scalar *>(data), size, msg.data.begin());
  publisher_->publish(msg);
}

Ros2Publisher::ArrayPublisher::PublisherVariant Ros2Publisher::ArrayPublisher::makePublisher(
    const rclcpp::Node::SharedPtr &node, const std::string &name, DataType dtype) {
  switch (dtype) {
    case DataType::kUndefined:
      STEPIT_THROW("Cannot create array channel '{}' with undefined data type.", name);
    case DataType::kFloat32:
      return Float32Publisher(node, name);
    case DataType::kInt32:
      return Int32Publisher(node, name);
    case DataType::kInt64:
      return Int64Publisher(node, name);
    case DataType::kBool:
      return BoolPublisher(node, name);
  }
  STEPIT_THROW("Unsupported data type '{}' for array channel '{}'.", dataTypeName(dtype), name);
}

Ros2Publisher::ArrayPublisher::ArrayPublisher(std::string name, DataType dtype)
    : name_(std::move(name)), dtype_(dtype), publisher_(makePublisher(getNode(), name_, dtype_)) {}

void Ros2Publisher::ArrayPublisher::publish(const void *data, DataType dtype, std::size_t size) {
  STEPIT_ASSERT(data != nullptr or size == 0, "Cannot publish non-empty array '{}' from a null pointer.", name_);
  STEPIT_ASSERT(dtype == dtype_, "Array channel '{}' is already registered as {}, cannot publish {}.", name_,
                dataTypeName(dtype_), dataTypeName(dtype));
  std::visit([data, size](auto &publisher) { publisher.publish(data, size); }, publisher_);
}

Ros2Publisher::Ros2Publisher() {
  auto node = getNode();
  if (publisher::g_filter.publish_status) {
    status_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("status", getDefaultQoS());
  }
  if (publisher::g_filter.publish_low_level) {
    imu_pub_   = node->create_publisher<sensor_msgs::msg::Imu>("imu", getDefaultQoS());
    joint_pub_ = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", getDefaultQoS());
  }
}

void Ros2Publisher::publishStatus() {
  if (not status_pub_) return;
  auto statuses     = getStatusSnapshot();
  status_msg_.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status_msg_.name  = "stepit";
  status_msg_.values.clear();
  for (const auto &item : statuses) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key   = item.first;
    kv.value = item.second;
    status_msg_.values.push_back(kv);
  }
  status_pub_->publish(status_msg_);
}

void Ros2Publisher::publishLowLevel(const RobotSpec &spec, const LowState &state, const LowCmd &cmd) {
  if (not imu_pub_ or not joint_pub_) return;
  auto timestamp                 = getNode()->now();
  imu_msg_.header.stamp          = timestamp;
  imu_msg_.angular_velocity.x    = state.imu.gyroscope[0];
  imu_msg_.angular_velocity.y    = state.imu.gyroscope[1];
  imu_msg_.angular_velocity.z    = state.imu.gyroscope[2];
  imu_msg_.linear_acceleration.x = state.imu.accelerometer[0];
  imu_msg_.linear_acceleration.y = state.imu.accelerometer[1];
  imu_msg_.linear_acceleration.z = state.imu.accelerometer[2];
  imu_msg_.orientation.w         = state.imu.quaternion[0];
  imu_msg_.orientation.x         = state.imu.quaternion[1];
  imu_msg_.orientation.y         = state.imu.quaternion[2];
  imu_msg_.orientation.z         = state.imu.quaternion[3];
  imu_pub_->publish(imu_msg_);

  joint_msg_.header.frame_id = spec.robot_name;
  joint_msg_.header.stamp    = timestamp;
  std::size_t msg_dim        = 3 * spec.dof + spec.foot_names.size();
  joint_msg_.position.resize(msg_dim);
  joint_msg_.velocity.resize(msg_dim);
  joint_msg_.effort.resize(msg_dim);
  joint_msg_.name.resize(msg_dim);

  std::size_t idx{};
  for (std::size_t i{}; i < spec.dof; ++i, ++idx) {
    const auto &joint_state  = state.motor_state[i];
    joint_msg_.name[idx]     = spec.joint_names[i] + "_joint";
    joint_msg_.position[idx] = joint_state.q;
    joint_msg_.velocity[idx] = joint_state.dq;
    joint_msg_.effort[idx]   = joint_state.tor;
  }
  for (std::size_t i{}; i < spec.foot_names.size(); ++i, ++idx) {
    joint_msg_.name[idx]     = spec.foot_names[i];
    joint_msg_.position[idx] = 0.0;
    joint_msg_.velocity[idx] = 0.0;
    joint_msg_.effort[idx]   = state.foot_force[i];
  }
  for (std::size_t i{}; i < spec.dof; ++i, ++idx) {
    const auto &joint_cmd    = cmd[i];
    joint_msg_.name[idx]     = spec.joint_names[i] + "_cmd";
    joint_msg_.position[idx] = joint_cmd.q;
    joint_msg_.velocity[idx] = joint_cmd.dq;
    joint_msg_.effort[idx]   = joint_cmd.tor;
  }
  for (std::size_t i{}; i < spec.dof; ++i, ++idx) {
    const auto &joint_state  = state.motor_state[i];
    const auto &joint_cmd    = cmd[i];
    joint_msg_.name[idx]     = spec.joint_names[i] + "_gain";
    joint_msg_.position[idx] = joint_cmd.Kp;
    joint_msg_.velocity[idx] = joint_cmd.Kd;
    joint_msg_.effort[idx]   =  // desired torque
        (joint_cmd.q - joint_state.q) * joint_cmd.Kp + (joint_cmd.dq - joint_state.dq) * joint_cmd.Kd + joint_cmd.tor;
  }
  joint_pub_->publish(joint_msg_);
}

void Ros2Publisher::publishArray(const std::string &name, const void *data, DataType dtype, std::size_t size) {
  STEPIT_ASSERT(data != nullptr or size == 0, "Cannot publish non-empty array '{}' from a null pointer.", name);
  array_publishers_.try_emplace(name, name, dtype).first->second.publish(data, dtype, size);
}

STEPIT_REGISTER_PUBLISHER(ros2, kDefPriority, Publisher::make<Ros2Publisher>);
}  // namespace stepit

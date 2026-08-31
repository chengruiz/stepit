#include <algorithm>
#include <utility>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>

#include <stepit/ros/node_handle.h>
#include <stepit/ros/publisher.h>

namespace stepit {
RosPublisher::ArrayPublisher::ArrayPublisher(std::string name, DataType dtype) : name_(std::move(name)), dtype_(dtype) {
  switch (dtype_) {
    case DataType::kUndefined:
      STEPIT_THROW("Cannot create array channel '{}' with undefined data type.", name_);
    case DataType::kFloat32:
      publisher_ = getNodeHandle().advertise<std_msgs::Float32MultiArray>(name_, 1);
      break;
    case DataType::kInt32:
      publisher_ = getNodeHandle().advertise<std_msgs::Int32MultiArray>(name_, 1);
      break;
    case DataType::kInt64:
      publisher_ = getNodeHandle().advertise<std_msgs::Int64MultiArray>(name_, 1);
      break;
    case DataType::kBool:
      publisher_ = getNodeHandle().advertise<std_msgs::UInt8MultiArray>(name_, 1);
      break;
    default:
      STEPIT_THROW("Unsupported data type '{}' for array channel '{}'.", dataTypeName(dtype_), name_);
  }
}

template <typename Message, typename Scalar>
void RosPublisher::ArrayPublisher::publishMessage(const void *data, std::size_t size) {
  Message msg;
  msg.data.resize(size);
  if (size > 0) std::copy_n(static_cast<const Scalar *>(data), size, msg.data.begin());
  publisher_.publish(msg);
}

void RosPublisher::ArrayPublisher::publish(const void *data, DataType dtype, std::size_t size) {
  STEPIT_ASSERT(data != nullptr or size == 0, "Cannot publish non-empty array '{}' from a null pointer.", name_);
  STEPIT_ASSERT(dtype == dtype_, "Array channel '{}' is already registered as {}, cannot publish {}.", name_,
                dataTypeName(dtype_), dataTypeName(dtype));
  switch (dtype_) {
    case DataType::kUndefined:
      STEPIT_THROW("Cannot publish array channel '{}' with undefined data type.", name_);
    case DataType::kFloat32:
      publishMessage<std_msgs::Float32MultiArray, float>(data, size);
      break;
    case DataType::kInt32:
      publishMessage<std_msgs::Int32MultiArray, std::int32_t>(data, size);
      break;
    case DataType::kInt64:
      publishMessage<std_msgs::Int64MultiArray, std::int64_t>(data, size);
      break;
    case DataType::kBool:
      publishMessage<std_msgs::UInt8MultiArray, std::uint8_t>(data, size);
      break;
    default:
      STEPIT_THROW("Unsupported data type '{}' for array channel '{}'.", dataTypeName(dtype_), name_);
  }
}

RosPublisher::RosPublisher() {
  if (publisher::g_filter.publish_status) {
    status_pub_ = getNodeHandle().advertise<diagnostic_msgs::DiagnosticStatus>("status", 1);
  }
  if (publisher::g_filter.publish_low_level) {
    imu_pub_   = getNodeHandle().advertise<sensor_msgs::Imu>("imu", 1);
    joint_pub_ = getNodeHandle().advertise<sensor_msgs::JointState>("joint_states", 1);
  }
}

void RosPublisher::publishStatus() {
  auto statuses     = getStatusSnapshot();
  status_msg_.level = diagnostic_msgs::DiagnosticStatus::OK;
  status_msg_.name  = "stepit";
  status_msg_.values.clear();
  for (const auto &item : statuses) {
    diagnostic_msgs::KeyValue kv_msg;
    kv_msg.key   = item.first;
    kv_msg.value = item.second;
    status_msg_.values.push_back(kv_msg);
  }
  status_pub_.publish(status_msg_);
}

void RosPublisher::publishLowLevel(const RobotSpec &spec, const LowState &state, const LowCmd &cmd) {
  auto timestamp                 = ros::Time::now();
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
  imu_pub_.publish(imu_msg_);

  joint_msg_.header.frame_id = spec.robot_name;
  joint_msg_.header.stamp    = timestamp;
  std::size_t msg_dim        = 3 * spec.dof + spec.foot_names.size();
  joint_msg_.position.resize(msg_dim, 0.);
  joint_msg_.velocity.resize(msg_dim, 0.);
  joint_msg_.effort.resize(msg_dim, 0.);
  joint_msg_.name.resize(msg_dim, {});

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
    joint_msg_.position[idx] = 0.;
    joint_msg_.velocity[idx] = 0.;
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
        joint_cmd.tor + (joint_cmd.q - joint_state.q) * joint_cmd.Kp + (joint_cmd.dq - joint_state.dq) * joint_cmd.Kd;
    ;
  }
  joint_pub_.publish(joint_msg_);
}

void RosPublisher::publishArray(const std::string &name, const void *data, DataType dtype, std::size_t size) {
  STEPIT_ASSERT(data != nullptr or size == 0, "Cannot publish non-empty array '{}' from a null pointer.", name);
  array_publishers_.try_emplace(name, name, dtype).first->second.publish(data, dtype, size);
}

STEPIT_REGISTER_PUBLISHER(ros, kDefPriority, Publisher::make<RosPublisher>);
}  // namespace stepit

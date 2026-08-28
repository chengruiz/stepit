#include <stepit/policy_neuro_ros2/field_subscriber2.h>
#include <stepit/ros2/node.h>

namespace stepit::neuro_policy {
FieldSubscriber2::FieldSubscriber2(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : Module(policy_spec, ModuleSpec(module_spec, "field_subscriber")) {
  config_.assertMap();
  using std_msgs::msg::Float32MultiArray;
  for (const auto &field_node : config_) {
    FieldData field;
    field_node.first.to(field.name);
    field_node.second["topic"].to(field.topic);
    field_node.second["size"].to(field.size);
    field_node.second["timeout_threshold"].to(field.timeout_threshold, true);
    field.id   = registerProvision(field.name, DataType::kFloat32, field.size);
    field.data = VecXf::Zero(static_cast<Eigen::Index>(field.size));

    std::size_t index = fields_.size();
    rclcpp::QoS qos   = parseQoS(field_node.second["qos"]);
    field.subscriber  = getNode()->create_subscription<Float32MultiArray>(
        field.topic, qos, [this, index](const Float32MultiArray::SharedPtr msg) { callback(index, msg); });
    field.received = false;
    fields_.push_back(std::move(field));
  }
}

bool FieldSubscriber2::reset() {
  std::lock_guard<std::mutex> _(mutex_);
  for (const auto &field : fields_) {
    if (not field.received) {
      STEPIT_WARN("Field '{}' is not received yet.", field.name);
      return false;
    }
  }
  return true;
}

bool FieldSubscriber2::update(const LowState &low_state, ControlRequests &requests, FieldMap &context) {
  std::lock_guard<std::mutex> _(mutex_);
  for (const auto &field : fields_) {
    if (field.timeout_threshold > 0.0 and getElapsedTime(field.stamp) > field.timeout_threshold) {
      STEPIT_WARN("Field '{}' has timed out.", field.name);
      return false;
    }
    if (field.data.size() != static_cast<Eigen::Index>(field.size)) {
      STEPIT_WARN("Field '{}' has unexpected size: expected {}, got {}.", field.name, field.size, field.data.size());
      return false;
    }
    context[field.id] = field.data;
  }
  return true;
}

void FieldSubscriber2::callback(std::size_t index, const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  std::lock_guard<std::mutex> _(mutex_);
  if (index >= fields_.size()) return;
  auto &field    = fields_[index];
  field.received = true;
  field.stamp    = getNode()->now();
  field.data     = VecXf::Map(msg->data.data(), static_cast<Eigen::Index>(msg->data.size()));
}

STEPIT_REGISTER_MODULE(field_subscriber, kDefPriority, Module::make<FieldSubscriber2>);
}  // namespace stepit::neuro_policy

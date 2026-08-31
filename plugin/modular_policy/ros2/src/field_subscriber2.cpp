#include <stepit/modular_policy_ros2/field_subscriber2.h>

#include <algorithm>
#include <utility>

#include <stepit/ros2/node.h>

namespace stepit::modular_policy {
FieldSubscriber2::FieldSubscriber2(const ModularPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : Module(policy_spec, ModuleSpec(module_spec, "field_subscriber")) {
  config_.assertMap();
  fields_.reserve(config_.size());
  for (const auto &field_node : config_) {
    FieldData field;
    const auto value_node = field_node.second;
    value_node.assertMap();
    field_node.first.to(field.name);
    value_node["topic"].to(field.topic);
    const auto size_node = value_node["size"];
    size_node.to(field.size);
    size_node.require(field.size > 0, "Expected a positive field size");
    field.dtype = parseDataType(value_node["dtype"].as<std::string>());
    value_node["timeout_threshold"].to(field.timeout_threshold, true);
    field.id = registerProvision(field.name, field.dtype, field.size);
    fields_.push_back(std::move(field));
  }

  std::size_t index{};
  for (const auto &field_node : config_) {
    const auto value_node = field_node.second;
    auto &field           = fields_.at(index);
    const rclcpp::QoS qos = parseQoS(value_node["qos"]);
    switch (field.dtype) {
      case DataType::kUndefined:
        STEPIT_THROW("Subscribed field '{}' cannot have undefined data type.", field.name);
      case DataType::kFloat32:
        field.subscriber = getNode()->create_subscription<std_msgs::msg::Float32MultiArray>(
            field.topic, qos,
            [this, index](const std_msgs::msg::Float32MultiArray::SharedPtr msg) { float32Callback(index, msg); });
        break;
      case DataType::kInt32:
        field.subscriber = getNode()->create_subscription<std_msgs::msg::Int32MultiArray>(
            field.topic, qos,
            [this, index](const std_msgs::msg::Int32MultiArray::SharedPtr msg) { int32Callback(index, msg); });
        break;
      case DataType::kInt64:
        field.subscriber = getNode()->create_subscription<std_msgs::msg::Int64MultiArray>(
            field.topic, qos,
            [this, index](const std_msgs::msg::Int64MultiArray::SharedPtr msg) { int64Callback(index, msg); });
        break;
      case DataType::kBool:
        field.subscriber = getNode()->create_subscription<std_msgs::msg::UInt8MultiArray>(
            field.topic, qos,
            [this, index](const std_msgs::msg::UInt8MultiArray::SharedPtr msg) { boolCallback(index, msg); });
        break;
    }
    ++index;
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
    if (not field.received) {
      STEPIT_WARN("Field '{}' is not received yet.", field.name);
      return false;
    }
    if (field.timeout_threshold > 0.0 and getElapsedTime(field.stamp) > field.timeout_threshold) {
      STEPIT_WARN("Field '{}' has timed out.", field.name);
      return false;
    }
    context[field.id] = field_values_.at(field.id);
  }
  return true;
}

template <typename Scalar, typename Sequence>
void FieldSubscriber2::storeField(std::size_t index, const Sequence &source) {
  FieldArray<Scalar> value(static_cast<Eigen::Index>(source.size()));
  std::transform(source.begin(), source.end(), value.data(),
                 [](const auto &item) { return static_cast<Scalar>(item); });
  const auto data_size = static_cast<std::size_t>(value.size());
  std::lock_guard<std::mutex> _(mutex_);
  if (index >= fields_.size()) return;
  auto &field = fields_[index];
  if (data_size != field.size) {
    field.received = false;
    STEPIT_WARN("Field '{}' has unexpected size: expected {}, got {}.", field.name, field.size, data_size);
    return;
  }

  field_values_[field.id] = std::move(value);
  field.stamp             = getNode()->now();
  field.received          = true;
}

void FieldSubscriber2::float32Callback(std::size_t index, const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  storeField<float>(index, msg->data);
}

void FieldSubscriber2::int32Callback(std::size_t index, const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
  storeField<std::int32_t>(index, msg->data);
}

void FieldSubscriber2::int64Callback(std::size_t index, const std_msgs::msg::Int64MultiArray::SharedPtr msg) {
  storeField<std::int64_t>(index, msg->data);
}

void FieldSubscriber2::boolCallback(std::size_t index, const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
  storeField<bool>(index, msg->data);
}

STEPIT_REGISTER_MODULE(field_subscriber, kDefPriority, Module::make<FieldSubscriber2>);
}  // namespace stepit::modular_policy

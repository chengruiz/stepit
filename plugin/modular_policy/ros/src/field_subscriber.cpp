#include <stepit/modular_policy_ros/field_subscriber.h>

#include <algorithm>
#include <utility>

namespace stepit {
namespace modular_policy {
FieldSubscriber::FieldSubscriber(const ModularPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : Module(policy_spec, ModuleSpec(module_spec, "field_subscriber")) {
  config_.assertMap();
  fields_.reserve(config_.size());
  for (auto it = config_.begin(); it != config_.end(); ++it) {
    FieldData field;
    const auto key_node   = it->first;
    const auto value_node = it->second;
    value_node.assertMap();
    key_node.to(field.name);
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
  for (auto it = config_.begin(); it != config_.end(); ++it, ++index) {
    const auto value_node      = it->second;
    auto &field                = fields_.at(index);
    const int queue_size       = value_node["queue_size"].as<int>(1);
    const auto transport_hints = parseTransportHints(value_node["transport_hints"]);
    switch (field.dtype) {
      case DataType::kUndefined:
        STEPIT_THROW("Subscribed field '{}' cannot have undefined data type.", field.name);
      case DataType::kFloat32:
        field.subscriber = getNodeHandle().subscribe<std_msgs::Float32MultiArray>(
            field.topic, queue_size,
            boost::bind(&FieldSubscriber::float32Callback, this, index, boost::placeholders::_1), ros::VoidConstPtr(),
            transport_hints);
        break;
      case DataType::kInt32:
        field.subscriber = getNodeHandle().subscribe<std_msgs::Int32MultiArray>(
            field.topic, queue_size, boost::bind(&FieldSubscriber::int32Callback, this, index, boost::placeholders::_1),
            ros::VoidConstPtr(), transport_hints);
        break;
      case DataType::kInt64:
        field.subscriber = getNodeHandle().subscribe<std_msgs::Int64MultiArray>(
            field.topic, queue_size, boost::bind(&FieldSubscriber::int64Callback, this, index, boost::placeholders::_1),
            ros::VoidConstPtr(), transport_hints);
        break;
      case DataType::kBool:
        field.subscriber = getNodeHandle().subscribe<std_msgs::UInt8MultiArray>(
            field.topic, queue_size, boost::bind(&FieldSubscriber::boolCallback, this, index, boost::placeholders::_1),
            ros::VoidConstPtr(), transport_hints);
        break;
    }
  }
}

bool FieldSubscriber::reset() {
  std::lock_guard<std::mutex> _(mutex_);
  for (const auto &field : fields_) {
    if (not field.received) {
      STEPIT_WARN("Field '{}' is not received yet.", field.name);
      return false;
    }
  }
  return true;
}

bool FieldSubscriber::update(const LowState &low_state, ControlRequests &requests, FieldMap &context) {
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
void FieldSubscriber::storeField(std::size_t index, const Sequence &source) {
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
  field.stamp             = ros::Time::now();
  field.received          = true;
}

void FieldSubscriber::float32Callback(std::size_t index, const std_msgs::Float32MultiArray::ConstPtr &msg) {
  storeField<float>(index, msg->data);
}

void FieldSubscriber::int32Callback(std::size_t index, const std_msgs::Int32MultiArray::ConstPtr &msg) {
  storeField<std::int32_t>(index, msg->data);
}

void FieldSubscriber::int64Callback(std::size_t index, const std_msgs::Int64MultiArray::ConstPtr &msg) {
  storeField<std::int64_t>(index, msg->data);
}

void FieldSubscriber::boolCallback(std::size_t index, const std_msgs::UInt8MultiArray::ConstPtr &msg) {
  storeField<bool>(index, msg->data);
}

STEPIT_REGISTER_MODULE(field_subscriber, kDefPriority, Module::make<FieldSubscriber>);
}  // namespace modular_policy
}  // namespace stepit

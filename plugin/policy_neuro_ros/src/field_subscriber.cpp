#include <stepit/policy_neuro_ros/field_subscriber.h>

namespace stepit {
namespace neuro_policy {
FieldSubscriber::FieldSubscriber(const PolicySpec &policy_spec, const std::string &home_dir)
    : config_(yml::loadFile(home_dir + "/field_subscriber.yml")) {
  STEPIT_ASSERT(config_.IsMap(), "'field_subscriber.yml' must contain a map of field configurations.");
  for (auto it = config_.begin(); it != config_.end(); ++it) {
    FieldData field;
    yml::setTo(it->first, field.name);
    yml::setTo(it->second, "topic", field.topic);
    yml::setTo(it->second, "size", field.size);
    yml::setIf(it->second, "timeout_threshold", field.timeout_threshold);
    field.id   = registerProvision(field.name, field.size);
    field.data = VecXf::Zero(static_cast<Eigen::Index>(field.size));

    int queue_size       = yml::readIf(it->second, "queue_size", 1);
    auto transport_hints = parseTransportHints(it->second["transport_hints"]);
    std::size_t index    = fields_.size();
    field.subscriber     = getNodeHandle().subscribe<std_msgs::Float32MultiArray>(
        field.topic, queue_size, boost::bind(&FieldSubscriber::callback, this, index, boost::placeholders::_1),
        ros::VoidConstPtr(), transport_hints);
    field.received = false;
    fields_.push_back(std::move(field));
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

bool FieldSubscriber::update(const LowState &low_state, ControlRequests &requests, FieldMap &result) {
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
    result[field.id] = field.data;
  }
  return true;
}

void FieldSubscriber::callback(std::size_t index, const std_msgs::Float32MultiArray::ConstPtr &msg) {
  std::lock_guard<std::mutex> _(mutex_);
  if (index >= fields_.size()) return;
  auto &field    = fields_[index];
  field.received = true;
  field.stamp    = ros::Time::now();
  field.data     = VecXf::Map(msg->data.data(), static_cast<Eigen::Index>(msg->data.size()));
}

STEPIT_REGISTER_MODULE(field_subscriber, kDefPriority, Module::make<FieldSubscriber>);
}  // namespace neuro_policy
}  // namespace stepit

#include <stepit/policy_neuro_redis/field_subscriber_redis.h>

#include <nlohmann/json.hpp>

namespace stepit {
namespace neuro_policy {
RedisFieldSubscriber::RedisFieldSubscriber(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : Module(policy_spec, ModuleSpec(module_spec, "redis_field_subscriber")) {
  config_.assertMap();
  config_.assertHasValue("connection", "fields");
  connection_   = redis::RedisClientConfig(config_["connection"]);
  redis_client_ = std::make_unique<redis::RedisClient>(name(), connection_);
  parseFields();
  STEPIT_ASSERT(not fields_.empty(), "Module '{}' requires at least one Redis field mapping.", name());
}

bool RedisFieldSubscriber::reset() {
  std::lock_guard<std::mutex> _(mutex_);
  fetchFields();
  for (const auto &field : fields_) {
    if (not field.received) {
      STEPIT_WARN("Field '{}' is not received yet.", field.name);
      return false;
    }
  }
  return true;
}

bool RedisFieldSubscriber::update(const LowState &low_state, ControlRequests &requests, FieldMap &context) {
  std::lock_guard<std::mutex> _(mutex_);
  fetchFields();
  for (const auto &field : fields_) {
    if (field.timeout_threshold > 0.0f and getElapsedSeconds(field.stamp) > field.timeout_threshold) {
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

void RedisFieldSubscriber::exit() {
  std::lock_guard<std::mutex> _(mutex_);
  redis_client_->disconnect();
}

void RedisFieldSubscriber::parseFields() {
  const auto fields_node = config_["fields"];
  fields_node.assertMap();
  for (const auto &field_node : fields_node) {
    addField(field_node.first, field_node.second);
  }
}

void RedisFieldSubscriber::addField(const yml::Node &key_node, const yml::Node &value_node) {
  value_node.assertMap();

  FieldData field;
  key_node.to(field.name);
  STEPIT_ASSERT(value_node["key"].hasValue(), "Redis field '{}' must specify 'key'.", field.name);

  value_node["key"].to(field.key);
  value_node["field"].to(field.json_field, true);
  value_node["size"].to(field.size);
  value_node["timeout_threshold"].to(field.timeout_threshold, true);

  STEPIT_ASSERT(field.size > 0, "Redis field '{}' must have a positive size, got {}.", field.name, field.size);
  if (field.json_field.empty()) field.json_field = field.name;

  field.id   = registerProvision(field.name, field.size);
  field.data = VecXf::Zero(static_cast<Eigen::Index>(field.size));
  fields_.push_back(std::move(field));
}

bool RedisFieldSubscriber::fetchFields() {
  bool updated = false;
  for (auto &field : fields_) {
    redis::RedisReadStatus status = fetchField(field);

    updated = (status == redis::RedisReadStatus::kOk) or updated;
    if (status == redis::RedisReadStatus::kError) break;
  }
  return updated;
}

redis::RedisReadStatus RedisFieldSubscriber::fetchField(FieldData &field) {
  VecXf data;
  redis::RedisClient::JsonDict payload;
  redis::RedisReadStatus status = redis_client_->get(field.key, payload);
  if (status != redis::RedisReadStatus::kOk) return status;
  if (not parseFieldValue(field, payload, data)) return redis::RedisReadStatus::kInvalidData;

  field.data     = std::move(data);
  field.received = true;
  field.stamp    = SteadyClock::now();
  return redis::RedisReadStatus::kOk;
}

bool RedisFieldSubscriber::parseFieldValue(const FieldData &field, const redis::RedisClient::JsonDict &payload,
                                           VecXf &data) const {
  try {
    auto it = payload.find(field.json_field);
    if (it == payload.end()) {
      STEPIT_WARN("Redis field '{}' ({}) is missing JSON member '{}'.", field.name, formatRedisField(field),
                  field.json_field);
      return false;
    }

    if (not it->is_array()) {
      STEPIT_WARN("Redis field '{}' ({}) must map to a JSON array, got {}.", field.name, formatRedisField(field),
                  it->type_name());
      return false;
    }

    if (it->size() != field.size) {
      STEPIT_WARN("Redis field '{}' ({}) has unexpected size: expected {}, got {} from '{}'.", field.name,
                  formatRedisField(field), field.size, it->size(), payload.dump());
      return false;
    }

    data.resize(static_cast<Eigen::Index>(it->size()));
    for (std::size_t i = 0; i < it->size(); ++i) {
      const auto &item = (*it)[i];
      if (not item.is_number()) {
        STEPIT_WARN("Redis field '{}' ({}) expects numeric JSON array elements, but index {} is {} in '{}'.",
                    field.name, formatRedisField(field), i, item.type_name(), payload.dump());
        return false;
      }
      data[static_cast<Eigen::Index>(i)] = item.get<float>();
    }
  } catch (const nlohmann::json::exception &err) {
    STEPIT_WARN("Failed to parse Redis field '{}' ({}): {}.", field.name, formatRedisField(field), err.what());
    return false;
  }
  return true;
}

std::string RedisFieldSubscriber::formatRedisField(const FieldData &field) const {
  return field.key + "." + field.json_field;
}

STEPIT_REGISTER_MODULE(redis_field_subscriber, kDefPriority, Module::make<RedisFieldSubscriber>);
}  // namespace neuro_policy
}  // namespace stepit

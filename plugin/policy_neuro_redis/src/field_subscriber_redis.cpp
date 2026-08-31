#include <stepit/policy_neuro_redis/field_subscriber_redis.h>

#include <cmath>
#include <cstdint>
#include <limits>

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
    if (field.data.dataType() != field.dtype or field.data.size() != field.size) {
      STEPIT_WARN("Field '{}' has unexpected data type or size: expected '{}' with size {}, got '{}' with size {}.",
                  field.name, dataTypeName(field.dtype), field.size, dataTypeName(field.data.dataType()),
                  field.data.size());
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
  field.dtype = parseDataType(value_node["dtype"].as<std::string>());
  value_node["timeout_threshold"].to(field.timeout_threshold, true);

  STEPIT_ASSERT(field.size > 0, "Redis field '{}' must have a positive size, got {}.", field.name, field.size);
  if (field.json_field.empty()) field.json_field = field.name;

  field.id   = registerProvision(field.name, field.dtype, field.size);
  field.data = FieldValue(FieldSpec{field.dtype, field.size});
  switch (field.dtype) {
    case DataType::kUndefined:
      STEPIT_UNREACHABLE();
    case DataType::kFloat32:
      field.data.view<float>().setZero();
      break;
    case DataType::kInt32:
      field.data.view<std::int32_t>().setZero();
      break;
    case DataType::kInt64:
      field.data.view<std::int64_t>().setZero();
      break;
    case DataType::kBool:
      field.data.view<bool>().setConstant(false);
      break;
  }
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
  FieldValue data;
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
                                           FieldValue &data) const {
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

    FieldValue parsed(FieldSpec{field.dtype, field.size});
    switch (field.dtype) {
      case DataType::kUndefined:
        STEPIT_WARN("Redis field '{}' ({}) has undefined data type.", field.name, formatRedisField(field));
        return false;
      case DataType::kFloat32: {
        auto values = parsed.view<float>();
        for (std::size_t i = 0; i < it->size(); ++i) {
          const auto &item = (*it)[i];
          if (not item.is_number()) {
            STEPIT_WARN("Redis field '{}' ({}) expects float32 JSON array elements, but index {} is {} in '{}'.",
                        field.name, formatRedisField(field), i, item.type_name(), payload.dump());
            return false;
          }
          const float value = item.get<float>();
          if (not std::isfinite(value)) {
            STEPIT_WARN("Redis field '{}' ({}) has non-finite float32 value at index {} in '{}'.", field.name,
                        formatRedisField(field), i, payload.dump());
            return false;
          }
          values[static_cast<Eigen::Index>(i)] = value;
        }
        break;
      }
      case DataType::kInt32: {
        auto values = parsed.view<std::int32_t>();
        for (std::size_t i = 0; i < it->size(); ++i) {
          const auto &item = (*it)[i];
          if (item.is_number_unsigned()) {
            const auto value = item.get<std::uint64_t>();
            if (value > static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max())) {
              STEPIT_WARN("Redis field '{}' ({}) has out-of-range int32 value at index {} in '{}'.", field.name,
                          formatRedisField(field), i, payload.dump());
              return false;
            }
            values[static_cast<Eigen::Index>(i)] = static_cast<std::int32_t>(value);
          } else if (item.is_number_integer()) {
            const auto value = item.get<std::int64_t>();
            if (value < std::numeric_limits<std::int32_t>::lowest() or
                value > std::numeric_limits<std::int32_t>::max()) {
              STEPIT_WARN("Redis field '{}' ({}) has out-of-range int32 value at index {} in '{}'.", field.name,
                          formatRedisField(field), i, payload.dump());
              return false;
            }
            values[static_cast<Eigen::Index>(i)] = static_cast<std::int32_t>(value);
          } else {
            STEPIT_WARN("Redis field '{}' ({}) expects int32 JSON array elements, but index {} is {} in '{}'.",
                        field.name, formatRedisField(field), i, item.type_name(), payload.dump());
            return false;
          }
        }
        break;
      }
      case DataType::kInt64: {
        auto values = parsed.view<std::int64_t>();
        for (std::size_t i = 0; i < it->size(); ++i) {
          const auto &item = (*it)[i];
          if (item.is_number_unsigned()) {
            const auto value = item.get<std::uint64_t>();
            if (value > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
              STEPIT_WARN("Redis field '{}' ({}) has out-of-range int64 value at index {} in '{}'.", field.name,
                          formatRedisField(field), i, payload.dump());
              return false;
            }
            values[static_cast<Eigen::Index>(i)] = static_cast<std::int64_t>(value);
          } else if (item.is_number_integer()) {
            values[static_cast<Eigen::Index>(i)] = item.get<std::int64_t>();
          } else {
            STEPIT_WARN("Redis field '{}' ({}) expects int64 JSON array elements, but index {} is {} in '{}'.",
                        field.name, formatRedisField(field), i, item.type_name(), payload.dump());
            return false;
          }
        }
        break;
      }
      case DataType::kBool: {
        auto values = parsed.view<bool>();
        for (std::size_t i = 0; i < it->size(); ++i) {
          const auto &item = (*it)[i];
          if (not item.is_boolean()) {
            STEPIT_WARN("Redis field '{}' ({}) expects bool JSON array elements, but index {} is {} in '{}'.",
                        field.name, formatRedisField(field), i, item.type_name(), payload.dump());
            return false;
          }
          values[static_cast<Eigen::Index>(i)] = item.get<bool>();
        }
        break;
      }
    }
    data = std::move(parsed);
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

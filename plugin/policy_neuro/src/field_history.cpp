#include <stepit/policy_neuro/field_history.h>

namespace stepit {
namespace neuro_policy {
FieldHistory::FieldHistory(const PolicySpec &policy_spec, const std::string &home_dir)
    : config_(yml::loadFile(home_dir + "/field_history.yml")) {
  STEPIT_ASSERT(config_.IsMap(), "'field_history.yml' must contain a map of field history configurations.");

  for (const auto &node : config_) {
    BufferConfig buffer;
    yml::setTo(node.first, buffer.target_name);
    STEPIT_ASSERT(node.second.IsMap(), "Definition for '{}' must be a map.", buffer.target_name);

    yml::setTo(node.second["source"], buffer.source_name);
    yml::setTo(node.second["history_len"], buffer.history_len);
    yml::setIf(node.second["newest_first"], buffer.newest_first);
    yml::setIf(node.second["default_value"], buffer.default_value);

    buffer.source_id = registerField(buffer.source_name, 0);
    buffer.target_id = registerProvision(buffer.target_name, 0);
    if (provisions_.find(buffer.source_id) == provisions_.end()) {
      registerRequirement(buffer.source_id);
    }
    buffers_.push_back(std::move(buffer));
  }
}

void FieldHistory::initFieldProperties() {
  for (auto &buffer : buffers_) {
    buffer.source_size = getFieldSize(buffer.source_id);
    STEPIT_ASSERT(buffer.source_size > 0, "Size of source field '{}' is undefined.", getFieldName(buffer.source_id));

    std::uint32_t target_size = buffer.source_size * buffer.history_len;
    setFieldSize(buffer.target_id, target_size);

    if (buffer.default_value.size() > 0) {
      STEPIT_ASSERT(buffer.default_value.size() == buffer.source_size,
                    "Default value size for '{}' does not match source field size.", buffer.target_name);
    } else {
      buffer.default_value = VecXf::Zero(buffer.source_size);
    }
    buffer.history.allocate(buffer.history_len);
    buffer.output_buffer.resize(target_size);
  }
}

bool FieldHistory::reset() {
  for (auto &buffer : buffers_) {
    buffer.history.fill(buffer.default_value);
  }
  return true;
}

bool FieldHistory::update(const LowState &, ControlRequests &, FieldMap &result) {
  for (auto &buffer : buffers_) {
    if (buffer.newest_first) {
      // newest -> oldest: frame_0 (newest), frame_1, ..., frame_(N-1) (oldest)
      buffer.history.push_front(result.at(buffer.source_id));
    } else {
      // oldest -> newest: frame_0 (oldest), frame_1, ..., frame_(N-1) (newest)
      buffer.history.push_back(result.at(buffer.source_id));
    }

    std::uint32_t offset = 0;
    for (const auto &frame : buffer.history) {
      stackField(frame, offset, buffer.output_buffer);
    }
    result[buffer.target_id] = buffer.output_buffer;
  }
  return true;
}

STEPIT_REGISTER_MODULE(field_history, kDefPriority, Module::make<FieldHistory>);
}  // namespace neuro_policy
}  // namespace stepit

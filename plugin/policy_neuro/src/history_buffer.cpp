#include <stepit/policy_neuro/history_buffer.h>

namespace stepit {
namespace neuro_policy {
HistoryBuffer::HistoryBuffer(const PolicySpec &policy_spec, const std::string &home_dir)
    : config_(yml::loadFile(home_dir + "/history_buffer.yml")) {
  STEPIT_ASSERT(config_.IsMap(), "'history_buffer.yml' must contain a map of history buffer configurations.");

  for (const auto &node : config_) {
    std::string target_field_name;
    yml::setTo(node.first, target_field_name);
    
    STEPIT_ASSERT(node.second.IsMap(), "Definition for '{}' must be a map.", target_field_name);

    BufferConfig buffer;
    std::string source_name = yml::readAs<std::string>(node.second["source"]);
    buffer.source_id = registerField(source_name, 0);
    yml::setTo(node.second["history_len"], buffer.history_len);
    yml::setIf(node.second["newest_first"], buffer.newest_first);
    
    if (provisions_.find(buffer.source_id) == provisions_.end()) {
      registerRequirement(buffer.source_id);
    }
    
    buffer.target_id = registerProvision(target_field_name, 0);
    buffers_.push_back(std::move(buffer));
  }
}

void HistoryBuffer::initFieldProperties() {
  std::uint32_t total_output_size = 0;
  
  for (auto &buffer : buffers_) {
    buffer.source_size = getFieldSize(buffer.source_id);
    STEPIT_ASSERT(buffer.source_size > 0, "Size of source field '{}' is undefined.", 
                  getFieldName(buffer.source_id));
    
    std::uint32_t target_size = buffer.source_size * buffer.history_len;
    setFieldSize(buffer.target_id, target_size);
    total_output_size = std::max(total_output_size, target_size);
    
    buffer.history.clear();
    buffer.history.resize(buffer.history_len, VecXf::Zero(buffer.source_size));
  }
  
  output_buffer_.resize(total_output_size);
}

bool HistoryBuffer::reset() {
  for (auto &buffer : buffers_) {
    buffer.history.clear();
    buffer.history.resize(buffer.history_len, VecXf::Zero(buffer.source_size));
  }
  return true;
}

bool HistoryBuffer::update(const LowState &, ControlRequests &, FieldMap &result) {
  for (auto &buffer : buffers_) {
    auto it = result.find(buffer.source_id);
    STEPIT_ASSERT(it != result.end(), "Source field '{}' not found in result map.", 
                  getFieldName(buffer.source_id));
    
    buffer.history.pop_back();
    buffer.history.push_front(it->second);
    
    std::uint32_t offset = 0;
    if (buffer.newest_first) {
      // newest -> oldest: frame_0 (newest), frame_1, ..., frame_(N-1) (oldest)
      for (const auto &frame : buffer.history) {
        output_buffer_.segment(offset, buffer.source_size) = frame;
        offset += buffer.source_size;
      }
    } else {
      // oldest -> newest: frame_0 (oldest), frame_1, ..., frame_(N-1) (newest)
      for (auto it = buffer.history.rbegin(); it != buffer.history.rend(); ++it) {
        output_buffer_.segment(offset, buffer.source_size) = *it;
        offset += buffer.source_size;
      }
    }
    
    result[buffer.target_id] = output_buffer_.head(buffer.source_size * buffer.history_len);
  }
  return true;
}

STEPIT_REGISTER_MODULE(history_buffer, kDefPriority, Module::make<HistoryBuffer>);
}  // namespace neuro_policy
}  // namespace stepit

#include <stepit/policy_neuro/field_history.h>

namespace stepit {
namespace neuro_policy {
FieldHistoryBuffer::FieldHistoryBuffer(const std::pair<YAML::Node, YAML::Node> &node) {
  yml::setTo(node.first, target_name_);
  const auto &config = node.second;
  STEPIT_ASSERT(config.IsMap(), "Definition for '{}' must be a map.", target_name_);

  yml::setTo(config["source"], source_name_);
  yml::setTo(config["history_len"], history_len_);
  STEPIT_ASSERT(history_len_ > 0, "History length for '{}' must be greater than 0.", target_name_);
  yml::setIf(config["newest_first"], newest_first_);
  yml::setIf(config["include_current_frame"], include_current_frame_);
  if (yml::hasValue(config, "default_value")) {
    yml::setTo(config, "default_value", default_value_);
  }
  source_id_ = registerField(source_name_, 0);
  target_id_ = registerField(target_name_, 0);
}

void FieldHistoryBuffer::initFieldProperties() {
  source_size_                = getFieldSize(source_id_);
  const FieldSize target_size = source_size_ * history_len_;
  setFieldSize(target_id_, target_size);

  if (default_value_.size() == 1) {
    default_value_ = VecXf::Constant(source_size_, default_value_[0]);
  } else if (default_value_.size() != 0) {
    STEPIT_ASSERT(default_value_.size() == source_size_,
                  "Default value size for '{}' does not match source field size.", target_name_);
  }
  history_.allocate(history_len_);
  output_.resize(target_size);
}

void FieldHistoryBuffer::push(const ArrXf &frame) {
  if (newest_first_) {
    // newest -> oldest: frame_0 (newest), frame_1, ..., frame_(N-1) (oldest)
    history_.push_front(frame);
  } else {
    // oldest -> newest: frame_0 (oldest), frame_1, ..., frame_(N-1) (newest)
    history_.push_back(frame);
  }
}

void FieldHistoryBuffer::updateOutput() {
  FieldSize offset = 0;
  for (const auto &frame : history_) {
    stackField(frame, offset, output_);
  }
  STEPIT_ASSERT(offset == output_.size(),
                "Output buffer size does not match total history size after stacking for '{}'.", target_name_);
}

const ArrXf &FieldHistoryBuffer::update(const ArrXf &frame) {
  if (history_.empty()) {
    history_.fill(default_value_.size() > 0 ? default_value_ : frame);
  }

  if (include_current_frame_) {
    push(frame);
    updateOutput();
  } else {
    updateOutput();
    push(frame);
  }
  return output_;
}

FieldHistory::FieldHistory(const NeuroPolicySpec &policy_spec, const std::string &name)
    : Module(policy_spec, nonEmptyOr(name, "field_history")) {
  STEPIT_ASSERT(config_.IsMap(), "'{}' must contain a map of field history configurations.", config_filename_);

  for (const auto &node : config_) {
    FieldHistoryBuffer buffer(node);
    registerRequirement(buffer.getSourceId());
    registerProvision(buffer.getTargetId());
    buffers_.push_back(std::move(buffer));
  }
}

void FieldHistory::initFieldProperties() {
  for (auto &buffer : buffers_) {
    buffer.initFieldProperties();
  }
}

bool FieldHistory::reset() {
  for (auto &buffer : buffers_) {
    buffer.clear();
  }
  return true;
}

bool FieldHistory::update(const LowState &, ControlRequests &, FieldMap &context) {
  for (auto &buffer : buffers_) {
    const auto &frame             = context.at(buffer.getSourceId());
    context[buffer.getTargetId()] = buffer.update(frame);
  }
  return true;
}

STEPIT_REGISTER_MODULE(field_history, kDefPriority, Module::make<FieldHistory>);
}  // namespace neuro_policy
}  // namespace stepit

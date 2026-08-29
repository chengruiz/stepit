#include <cstring>
#include <set>

#include <stepit/field/operator.h>

namespace stepit {
namespace field {
AffineOperator::AffineOperator(const yml::Node &config) : node_(config) {
  if (config["field"].hasValue()) {
    const auto field_name = config["field"].as<std::string>();
    source_id_            = registerRequirement(field_name, DataType::kFloat32);
    target_id_            = source_id_;
  } else {
    config.require(config["source"].hasValue() and config["target"].hasValue(),
                   "Specify 'field' or both 'source' and 'target' for 'affine' operator");
    source_id_ = registerRequirement(config["source"].as<std::string>(), DataType::kFloat32);
    target_id_ = registerProvision(config["target"].as<std::string>(), DataType::kFloat32);
  }

  config.assertMutuallyExclusive({"scale", "std"});
  config.assertMutuallyExclusive({"bias", "mean"});
}

void AffineOperator::init() {
  const FieldSpec source_spec = getFieldSpec(source_id_);
  setFieldSpec(target_id_, source_spec);
  field_size_ = source_spec.size;
  scale_      = ArrXf::Ones(field_size_);
  bias_       = ArrXf::Zero(field_size_);

  const auto scale_node = node_["scale"];
  const auto std_node   = node_["std"];
  if (scale_node.hasValue()) {
    scale_node.to(scale_);
  } else if (std_node.hasValue()) {
    ArrXf std{ArrXf::Ones(field_size_)};
    std_node.to(std);
    std_node.require((std > kEPS).all(), "'std' values must be positive");
    scale_ = std.cwiseInverse();
  }

  const auto bias_node = node_["bias"];
  const auto mean_node = node_["mean"];
  if (bias_node.hasValue()) {
    bias_node.to(bias_);
  } else if (mean_node.hasValue()) {
    ArrXf mean{ArrXf::Zero(field_size_)};
    mean_node.to(mean);
    bias_ = -mean.cwiseProduct(scale_);
  }
}

bool AffineOperator::update(FieldMap &context) {
  ArrXf transformed   = readFieldValue(context, source_id_).get<float>().cwiseProduct(scale_) + bias_;
  context[target_id_] = transformed;
  return true;
}

ConcatOperator::ConcatOperator(const yml::Node &config) {
  config.assertHasValue("sources", "target");
  const auto target_name = config["target"].as<std::string>();
  auto sources_node      = config["sources"];
  sources_node.assertSequence();
  for (const auto &source_node : sources_node) {
    const auto source_name = source_node.as<std::string>();
    source_node.throwIf(source_name == target_name, "Concat target must not also be a source");
    source_ids_.push_back(registerRequirement(source_name));
  }
  sources_node.require(not source_ids_.empty(), "'sources' must not be empty");
  target_id_ = registerProvision(target_name);
}

void ConcatOperator::init() {
  FieldSize target_size{};
  DataType dtype{DataType::kUndefined};
  source_bytes_.clear();
  source_bytes_.reserve(source_ids_.size());
  for (FieldId source_id : source_ids_) {
    const FieldSpec source_spec = getFieldSpec(source_id);
    if (dtype == DataType::kUndefined) {
      dtype = source_spec.dtype;
    } else {
      STEPIT_ASSERT(dtype == source_spec.dtype, "Cannot concatenate fields '{}' and '{}' with data types {} and {}.",
                    getFieldName(source_ids_.front()), getFieldName(source_id), dataTypeName(dtype),
                    dataTypeName(source_spec.dtype));
    }
    target_size += source_spec.size;
    source_bytes_.push_back(source_spec.byteSize());
  }
  const FieldSpec target_spec{dtype, target_size};
  setFieldSpec(target_id_, target_spec);
  target_bytes_ = target_spec.byteSize();
}

bool ConcatOperator::update(FieldMap &context) {
  auto *target       = ensureFieldValue(context, target_id_).data();
  std::size_t offset = 0;
  for (std::size_t i{}; i < source_ids_.size(); ++i) {
    std::memcpy(target + offset, readFieldValue(context, source_ids_[i]).data(), source_bytes_[i]);
    offset += source_bytes_[i];
  }
  STEPIT_ASSERT(offset == target_bytes_, "Concat copied {} bytes, expected {}.", offset, target_bytes_);
  return true;
}

ConstOperator::ConstOperator(const yml::Node &config) {
  config.require(config["target"].hasValue() or config["field"].hasValue(),
                 "Specify 'target' or 'field' for 'const' operator");
  auto target_name = config["target"].hasValue() ? config["target"].as<std::string>()
                                                 : config["field"].as<std::string>();

  auto value_node = config["value"];
  value_node
      .require(value_node.isScalar() or value_node.isNonEmptySequence(), "Expected a scalar or a non-empty sequence");

  FieldSize size{};
  if (value_node.isScalar()) {
    config["size"].to(size);
    value_.setZero(size);
    value_node.to(value_);
  } else {
    value_node.to(value_);
    config["size"].to(size, true);
    if (size > 0) {
      config.throwIf(value_.size() != size, "'size' and 'value' lengths mismatch");
    } else {
      size = static_cast<std::size_t>(value_.size());
    }
  }

  target_id_ = registerProvision(target_name, DataType::kFloat32, size);
}

bool ConstOperator::update(FieldMap &context) {
  context[target_id_] = value_;
  return true;
}

CopyOperator::CopyOperator(const yml::Node &config) {
  config.assertHasValue("source", "target");
  auto source_name = config["source"].as<std::string>();
  auto target_name = config["target"].as<std::string>();
  config.throwIf(source_name == target_name, "'source' and 'target' must not be the same");
  source_id_ = registerRequirement(source_name);
  target_id_ = registerProvision(target_name);
}

void CopyOperator::init() { setFieldSpec(target_id_, getFieldSpec(source_id_)); }

bool CopyOperator::update(FieldMap &context) {
  context[target_id_] = readFieldValue(context, source_id_);
  return true;
}

HistoryOperator::HistoryOperator(const yml::Node &config) {
  config.assertHasValue("source", "target");
  auto source_name = config["source"].as<std::string>();
  auto target_name = config["target"].as<std::string>();
  source_size_     = config["source_size"].as<FieldSize>(0);
  if (config["source_size"].hasValue()) {
    config["source_size"].require(source_size_ > 0, "'source_size' must be greater than 0");
  }
  history_len_ = config["history_len"].as<std::uint32_t>();
  config["history_len"].require(history_len_ > 0, "'history_len' must be greater than 0");
  config.to(indices_);
  newest_first_          = config["newest_first"].as<bool>(true);
  include_current_frame_ = config["include_current_frame"].as<bool>(true);
  config.throwIf(source_name == target_name, "'source' and 'target' must not be the same");
  if (config["default_value"].isDefined()) {
    if (config["default_value"].hasValue()) config["default_value"].to(default_value_);
  } else {  // Fill with zeros by default if not provided
    default_value_ = ArrXf::Zero(1);
  }

  source_id_ = registerField(source_name, DataType::kFloat32, source_size_);
  if (include_current_frame_ or default_value_.size() == 0) {
    registerRequirement(source_id_);
  }  // otherwise, skip requirement registration since the field is not needed at update time
  target_id_ = registerProvision(target_name, DataType::kFloat32);
}

void HistoryOperator::init() {
  if (target_size_ > 0) return;
  source_size_ = getFieldSize(source_id_);
  indices_.canonicalize(history_len_);
  target_size_ = source_size_ * indices_.size();
  setFieldSpec(target_id_, FieldSpec{DataType::kFloat32, target_size_});

  if (default_value_.size() == 1) {
    default_value_ = VecXf::Constant(source_size_, default_value_[0]);
  } else if (default_value_.size() != 0) {
    STEPIT_ASSERT(default_value_.size() == source_size_,
                  "Default value size of history op does not match the source field size.");
  }

  history_.allocate(history_len_);
  output_.resize(target_size_);
}

bool HistoryOperator::reset() {
  history_.clear();
  if (default_value_.size() > 0) {
    history_.fill(default_value_);
    updateOutput();
  }
  return true;
}

void HistoryOperator::push(const ArrXf &frame) {
  if (newest_first_) {
    history_.push_front(frame);
  } else {
    history_.push_back(frame);
  }
}

void HistoryOperator::updateOutput() {
  FieldSize offset = 0;
  for (auto index : indices_) {
    stackField(history_[index], offset, output_);
  }
  STEPIT_ASSERT(offset == output_.size(), "History field size ({}) does not match the target size ({}).", offset,
                output_.size());
}

bool HistoryOperator::update(FieldMap &context) {
  if (history_.empty()) {
    history_.fill(context.at(source_id_).get<float>());
    updateOutput();
  }

  if (include_current_frame_) {
    push(context.at(source_id_).get<float>());
    updateOutput();
  }

  context[target_id_] = output_;
  return true;
}

void HistoryOperator::postStep(const FieldMap &context) {
  if (not include_current_frame_) {
    auto it = context.find(source_id_);
    STEPIT_ASSERT(it != context.end(), "Field '{}' not found at runtime.", getFieldName(source_id_));
    push(it->second.get<float>());
    updateOutput();
  }
}

MaskedFillOperator::MaskedFillOperator(const yml::Node &config) {
  if (config["field"].hasValue()) {
    const auto field_name = config["field"].as<std::string>();
    source_id_            = registerRequirement(field_name, DataType::kFloat32);
    target_id_            = source_id_;
  } else {
    config.require(config["source"].hasValue() and config["target"].hasValue(),
                   "Specify 'field' or both 'source' and 'target' for 'masked_fill' operator");
    source_id_ = registerRequirement(config["source"].as<std::string>(), DataType::kFloat32);
    target_id_ = registerProvision(config["target"].as<std::string>(), DataType::kFloat32);
  }
  config.to(indices_);
  config["value"].to(value_, true);
}

void MaskedFillOperator::init() {
  const FieldSpec source_spec = getFieldSpec(source_id_);
  setFieldSpec(target_id_, source_spec);
  field_size_ = source_spec.size;
  indices_.canonicalize(field_size_);
  buffer_.resize(field_size_);
}

bool MaskedFillOperator::update(FieldMap &context) {
  buffer_ = readFieldValue(context, source_id_).get<float>();
  for (auto index : indices_) buffer_[index] = value_;
  context[target_id_] = buffer_;
  return true;
}

SliceOperator::SliceOperator(const yml::Node &config) {
  config.require(config["source"].hasValue() and config["target"].hasValue(),
                 "Slice op must contain 'source' and 'target'");
  const auto source_name = config["source"].as<std::string>();
  const auto target_name = config["target"].as<std::string>();
  config.throwIf(source_name == target_name, "'source' and 'target' must not be the same");
  source_id_ = registerRequirement(source_name);
  target_id_ = registerProvision(target_name);
  config.to(indices_);
}

void SliceOperator::init() {
  const FieldSpec source_spec = getFieldSpec(source_id_);
  indices_.canonicalize(source_spec.size);
  setFieldSpec(target_id_, FieldSpec{source_spec.dtype, indices_.size()});
  element_size_ = dataTypeSize(source_spec.dtype);
  source_bytes_ = source_spec.byteSize();
}

bool SliceOperator::update(FieldMap &context) {
  const auto *source = readFieldValue(context, source_id_).data();
  auto *target       = ensureFieldValue(context, target_id_).data();
  for (std::size_t i{}; i < indices_.size(); ++i) {
    const std::size_t source_offset = indices_[i] * element_size_;
    STEPIT_ASSERT(source_offset + element_size_ <= source_bytes_, "Slice source byte range [{}..{}) exceeds {}.",
                  source_offset, source_offset + element_size_, source_bytes_);
    std::memcpy(target + i * element_size_, source + source_offset, element_size_);
  }
  return true;
}

SplitOperator::SplitOperator(const yml::Node &config) {
  config.assertHasValue("source", "targets");
  const auto source_name = config["source"].as<std::string>();
  source_id_             = registerRequirement(source_name);

  const auto targets_node = config["targets"];
  targets_node.assertSequence();
  std::set<std::string> target_names;
  std::size_t total_size{};
  for (const auto &target_node : targets_node) {
    target_node.assertMap();
    const auto name = target_node["name"].as<std::string>();
    const auto size = target_node["size"].as<FieldSize>();
    target_node["size"].require(size > 0, "Split target size must be greater than 0");
    target_node.throwIf(name == source_name, "Split target must not be the same as its source");
    target_node.throwIf(not target_names.insert(name).second, fmt::format("Duplicate split target '{}'", name));
    target_ids_.push_back(registerProvision(name, DataType::kUndefined, size));
    segment_sizes_.push_back(size);
    total_size += size;
  }
  targets_node.require(not target_ids_.empty(), "'targets' must not be empty");
  registerField(source_name, DataType::kUndefined, total_size);
}

void SplitOperator::init() {
  const FieldSpec source_spec = getFieldSpec(source_id_);
  std::size_t total_size{};
  segment_bytes_.clear();
  segment_bytes_.reserve(segment_sizes_.size());
  for (std::size_t i{}; i < segment_sizes_.size(); ++i) {
    total_size += segment_sizes_[i];
    const FieldSpec target_spec{source_spec.dtype, segment_sizes_[i]};
    setFieldSpec(target_ids_[i], target_spec);
    segment_bytes_.push_back(target_spec.byteSize());
  }
  STEPIT_ASSERT(total_size == source_spec.size, "Split sizes ({}) do not match source size ({}) for '{}'.", total_size,
                source_spec.size, getFieldName(source_id_));
  source_bytes_ = source_spec.byteSize();
}

bool SplitOperator::update(FieldMap &context) {
  const auto *source = readFieldValue(context, source_id_).data();
  std::size_t offset = 0;
  for (std::size_t i{}; i < target_ids_.size(); ++i) {
    std::memcpy(ensureFieldValue(context, target_ids_[i]).data(), source + offset, segment_bytes_[i]);
    offset += segment_bytes_[i];
  }
  STEPIT_ASSERT(offset == source_bytes_, "Split copied {} bytes, expected {}.", offset, source_bytes_);
  return true;
}

STEPIT_REGISTER_FIELD_OPERATOR(affine, kDefPriority, Operator::make<AffineOperator>);
STEPIT_REGISTER_FIELD_OPERATOR(concat, kDefPriority, Operator::make<ConcatOperator>);
STEPIT_REGISTER_FIELD_OPERATOR(const, kDefPriority, Operator::make<ConstOperator>);
STEPIT_REGISTER_FIELD_OPERATOR(copy, kDefPriority, Operator::make<CopyOperator>);
STEPIT_REGISTER_FIELD_OPERATOR(masked_fill, kDefPriority, Operator::make<MaskedFillOperator>);
STEPIT_REGISTER_FIELD_OPERATOR(slice, kDefPriority, Operator::make<SliceOperator>);
STEPIT_REGISTER_FIELD_OPERATOR(history, kDefPriority, Operator::make<HistoryOperator>);
STEPIT_REGISTER_FIELD_OPERATOR(split, kDefPriority, Operator::make<SplitOperator>);
}  // namespace field
}  // namespace stepit

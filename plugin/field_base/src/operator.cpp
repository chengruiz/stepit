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

CastOperator::CastOperator(const yml::Node &config) {
  config.assertHasValue("source", "target", "dtype");
  const auto source_name = config["source"].as<std::string>();
  const auto target_name = config["target"].as<std::string>();
  config.throwIf(source_name == target_name, "'source' and 'target' must not be the same");

  target_dtype_ = parseDataType(config["dtype"].as<std::string>());
  source_id_    = registerRequirement(source_name);
  target_id_    = registerProvision(target_name, target_dtype_);
}

void CastOperator::init() {
  const FieldSpec source_spec = getFieldSpec(source_id_);
  setFieldSpec(target_id_, FieldSpec{target_dtype_, source_spec.size});
}

bool CastOperator::update(FieldMap &context) {
  context[target_id_] = readFieldValue(context, source_id_).cast(target_dtype_);
  return true;
}

ConcatOperator::ConcatOperator(const yml::Node &config) {
  config.assertHasValue("sources", "target");
  const auto target_name = config["target"].as<std::string>();
  const auto sources_node = config["sources"];
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
  }
  setFieldSpec(target_id_, FieldSpec{dtype, target_size});
}

bool ConcatOperator::update(FieldMap &context) {
  auto &target            = ensureFieldValue(context, target_id_);
  FieldSize target_offset = 0;
  for (FieldId source_id : source_ids_) {
    const auto &source = readFieldValue(context, source_id);
    target.copyFrom(source, 0, target_offset, source.size());
    target_offset += source.size();
  }
  STEPIT_ASSERT(target_offset == target.size(), "Concat copied {} elements, expected {}.", target_offset,
                target.size());
  return true;
}

ConstOperator::ConstOperator(const yml::Node &config) {
  config.require(config["target"].hasValue() or config["field"].hasValue(),
                 "Specify 'target' or 'field' for 'const' operator");
  const auto target_name = config["target"].hasValue() ? config["target"].as<std::string>()
                                                       : config["field"].as<std::string>();

  const auto value_node = config["value"];
  value_node.require(value_node.isScalar() or value_node.isNonEmptySequence(),
                     "Expected a scalar or a non-empty sequence");

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
  const auto source_name = config["source"].as<std::string>();
  const auto target_name = config["target"].as<std::string>();
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
  const auto source_name = config["source"].as<std::string>();
  const auto target_name = config["target"].as<std::string>();
  source_size_           = config["source_size"].as<FieldSize>(0);
  if (config["source_size"].hasValue()) {
    config["source_size"].require(source_size_ > 0, "'source_size' must be greater than 0");
  }
  history_len_ = config["history_len"].as<std::uint32_t>();
  config["history_len"].require(history_len_ > 0, "'history_len' must be greater than 0");
  config.to(indices_);
  newest_first_          = config["newest_first"].as<bool>(true);
  include_current_frame_ = config["include_current_frame"].as<bool>(true);
  config.throwIf(source_name == target_name, "'source' and 'target' must not be the same");

  default_value_node_ = config["default_value"];
  if (default_value_node_.isDefined()) {
    default_value_node_.require(
        not default_value_node_.hasValue() or default_value_node_.isScalar() or default_value_node_.isSequence(),
        "Expected 'default_value' to be a scalar, sequence, or null");
  }
  has_default_value_ = not default_value_node_.isDefined() or
                       (default_value_node_.hasValue() and
                        (not default_value_node_.isSequence() or default_value_node_.size() > 0));

  source_id_ = registerField(source_name, DataType::kUndefined, source_size_);
  if (include_current_frame_ or not has_default_value_) registerRequirement(source_id_);
  target_id_ = registerProvision(target_name);
}

void HistoryOperator::init() {
  const FieldSpec source_spec = getFieldSpec(source_id_);
  indices_.canonicalize(history_len_);
  source_size_ = source_spec.size;
  target_size_ = source_size_ * indices_.size();
  setFieldSpec(target_id_, FieldSpec{source_spec.dtype, target_size_});

  if (has_default_value_) {
    const yml::Node value_node =
        default_value_node_.isSequence(1) ? default_value_node_[0] : default_value_node_;
    default_value_ = parseFieldValue(value_node, source_spec, true);
  }
  history_.allocate(history_len_);
}

bool HistoryOperator::reset() {
  history_.clear();
  if (has_default_value_) history_.fill(default_value_);
  return true;
}

void HistoryOperator::push(const FieldValue &frame) {
  if (newest_first_) {
    history_.push_front(frame);
  } else {
    history_.push_back(frame);
  }
}

void HistoryOperator::render(FieldValue &target) const {
  FieldSize target_offset = 0;
  for (auto index : indices_) {
    const auto &frame = history_[index];
    STEPIT_ASSERT(frame.size() == source_size_ and
                      frame.dataType() == getFieldDataType(source_id_),
                  "History frame specification does not match source field '{}'.", getFieldName(source_id_));
    target.copyFrom(frame, 0, target_offset, frame.size());
    target_offset += frame.size();
  }
  STEPIT_ASSERT(target_offset == target.size(), "History rendered {} elements, expected {}.", target_offset,
                target.size());
}

bool HistoryOperator::update(FieldMap &context) {
  const FieldValue *source = nullptr;
  if (history_.empty() or include_current_frame_) {
    source = &readFieldValue(context, source_id_);
  }
  if (history_.empty()) history_.fill(*source);
  if (include_current_frame_) push(*source);
  render(ensureFieldValue(context, target_id_));
  return true;
}

void HistoryOperator::postStep(const FieldMap &context) {
  if (not include_current_frame_) push(readFieldValue(context, source_id_));
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
}

bool SliceOperator::update(FieldMap &context) {
  const auto &source = readFieldValue(context, source_id_);
  auto &target       = ensureFieldValue(context, target_id_);
  for (std::size_t i{}; i < indices_.size(); ++i) {
    target.copyFrom(source, indices_[i], i, 1);
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
  FieldSize total_size{};
  for (std::size_t i{}; i < segment_sizes_.size(); ++i) {
    total_size += segment_sizes_[i];
    setFieldSpec(target_ids_[i], FieldSpec{source_spec.dtype, segment_sizes_[i]});
  }
  STEPIT_ASSERT(total_size == source_spec.size, "Split sizes ({}) do not match source size ({}) for '{}'.", total_size,
                source_spec.size, getFieldName(source_id_));
}

bool SplitOperator::update(FieldMap &context) {
  const auto &source       = readFieldValue(context, source_id_);
  FieldSize source_offset = 0;
  for (std::size_t i{}; i < target_ids_.size(); ++i) {
    auto &target = ensureFieldValue(context, target_ids_[i]);
    target.copyFrom(source, source_offset, 0, target.size());
    source_offset += target.size();
  }
  STEPIT_ASSERT(source_offset == source.size(), "Split copied {} elements, expected {}.", source_offset,
                source.size());
  return true;
}

STEPIT_REGISTER_FIELD_OPERATOR(affine, kDefPriority, Operator::make<AffineOperator>);
STEPIT_REGISTER_FIELD_OPERATOR(cast, kDefPriority, Operator::make<CastOperator>);
STEPIT_REGISTER_FIELD_OPERATOR(concat, kDefPriority, Operator::make<ConcatOperator>);
STEPIT_REGISTER_FIELD_OPERATOR(const, kDefPriority, Operator::make<ConstOperator>);
STEPIT_REGISTER_FIELD_OPERATOR(copy, kDefPriority, Operator::make<CopyOperator>);
STEPIT_REGISTER_FIELD_OPERATOR(masked_fill, kDefPriority, Operator::make<MaskedFillOperator>);
STEPIT_REGISTER_FIELD_OPERATOR(slice, kDefPriority, Operator::make<SliceOperator>);
STEPIT_REGISTER_FIELD_OPERATOR(history, kDefPriority, Operator::make<HistoryOperator>);
STEPIT_REGISTER_FIELD_OPERATOR(split, kDefPriority, Operator::make<SplitOperator>);
}  // namespace field
}  // namespace stepit

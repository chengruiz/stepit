#include <numeric>

#include <stepit/field/operator.h>

namespace stepit {
namespace field {
AffineOperator::AffineOperator(const yml::Node &config) : node_(config) {
  if (config["field"].hasValue()) {
    auto field_name = config["field"].as<std::string>();
    source_id_      = registerRequirement(field_name);
    target_id_      = source_id_;
  } else {
    STEPIT_ASSERT(config["source"].hasValue() and config["target"].hasValue(),
                  "Affine op must contain 'field' or both 'source' and 'target'.");
    auto source_name = config["source"].as<std::string>();
    auto target_name = config["target"].as<std::string>();
    source_id_       = registerRequirement(source_name);
    target_id_       = registerProvision(target_name, 0);
  }

  config.assertMutuallyExclusive({"scale", "std"});
  config.assertMutuallyExclusive({"bias", "mean"});

  try {
    init();
  } catch (const UndefinedFieldSizeError &) {}
}

void AffineOperator::init() {
  if (field_size_ > 0) return;
  field_size_ = getFieldSize(source_id_);
  setFieldSize(target_id_, field_size_);
  scale_ = ArrXf::Ones(field_size_);
  bias_  = ArrXf::Zero(field_size_);

  if (node_["scale"].hasValue()) {
    node_["scale"].to(scale_);
  } else if (node_["std"].hasValue()) {
    ArrXf std{ArrXf::Ones(field_size_)};
    node_["std"].to(std);
    STEPIT_ASSERT((std > kEPS).all(), "'std' values of affine op must be positive.");
    scale_ = std.cwiseInverse();
  }

  if (node_["bias"].hasValue()) {
    node_["bias"].to(bias_);
  } else if (node_["mean"].hasValue()) {
    ArrXf mean{ArrXf::Zero(field_size_)};
    node_["mean"].to(mean);
    bias_ = -mean.cwiseProduct(scale_);
  }
}

bool AffineOperator::update(FieldMap &context) {
  auto transformed    = context.at(source_id_).cwiseProduct(scale_) + bias_;
  context[target_id_] = std::move(transformed);
  return true;
}

ConcatOperator::ConcatOperator(const yml::Node &config) {
  config.assertHasValue("sources", "target");
  auto sources_node = config["sources"];
  sources_node.assertSequence();
  for (const auto &source_node : sources_node) {
    source_ids_.push_back(registerRequirement(source_node.as<std::string>()));
  }
  target_id_ = registerProvision(config["target"].as<std::string>(), 0);

  try {
    init();
  } catch (const UndefinedFieldSizeError &) {}
}

void ConcatOperator::init() {
  if (target_size_ > 0) return;
  FieldSize total_size = 0;
  for (auto source_id : source_ids_) {
    total_size += getFieldSize(source_id);
  }
  setFieldSize(target_id_, total_size);
  target_size_ = total_size;
  buffer_.resize(total_size);
}

bool ConcatOperator::update(FieldMap &context) {
  concatFields(context, source_ids_, buffer_);
  context[target_id_] = buffer_;
  return true;
}

ConstOperator::ConstOperator(const yml::Node &config) {
  STEPIT_ASSERT(config["target"].hasValue() or config["field"].hasValue(),
                "Const op must contain 'target' or 'field'.");
  STEPIT_ASSERT(config["value"].hasValue(), "Const op must contain 'value'.");

  auto target_name = config["target"].hasValue() ? config["target"].as<std::string>()
                                                 : config["field"].as<std::string>();

  auto value_node = config["value"];
  STEPIT_ASSERT(value_node.isScalar() or value_node.isSequence(), "Const op 'value' must be a scalar or sequence.");

  FieldSize size{};
  if (value_node.isScalar()) {
    config["size"].to(size);
    value_.setZero(size);
    value_node.to(value_);
  } else {
    value_node.to(value_);
    config["size"].to(size, true);
    if (size > 0) {
      STEPIT_ASSERT(value_.size() == size, "Const op has mismatched 'size' and 'value' lengths.");
    } else {
      size = static_cast<FieldSize>(value_.size());
    }
  }

  target_id_ = registerProvision(target_name, size);
}

bool ConstOperator::update(FieldMap &context) {
  context[target_id_] = value_;
  return true;
}

CopyOperator::CopyOperator(const yml::Node &config) {
  config.assertHasValue("source", "target");
  auto source_name = config["source"].as<std::string>();
  auto target_name = config["target"].as<std::string>();
  STEPIT_ASSERT(source_name != target_name, "Source and target cannot be the same in a copy op.");
  source_id_ = registerRequirement(source_name);
  target_id_ = registerProvision(target_name, 0);

  try {
    init();
  } catch (const UndefinedFieldSizeError &) {}
}

void CopyOperator::init() {
  if (field_size_ > 0) return;
  field_size_ = getFieldSize(source_id_);
  setFieldSize(target_id_, field_size_);
}

bool CopyOperator::update(FieldMap &context) {
  context[target_id_] = context.at(source_id_);
  return true;
}

HistoryOperator::HistoryOperator(const yml::Node &config) {
  config.assertHasValue("source", "target");
  auto source_name = config["source"].as<std::string>();
  auto target_name = config["target"].as<std::string>();
  source_size_     = config["source_size"].as<FieldSize>(0);
  history_len_     = config["history_len"].as<std::uint32_t>();
  STEPIT_ASSERT(history_len_ > 0, "'history_len' of history op must be greater than 0.");
  newest_first_          = config["newest_first"].as<bool>(true);
  include_current_frame_ = config["include_current_frame"].as<bool>(true);
  if (config["default_value"].isDefined()) {
    if (config["default_value"].hasValue()) config["default_value"].to(default_value_);
  } else {  // Fill with zeros by default if not provided
    default_value_ = ArrXf::Zero(1);
  }

  source_id_ = registerField(source_name, source_size_);
  if (include_current_frame_ or default_value_.size() == 0) {
    registerRequirement(source_id_);
  }  // otherwise, skip requirement registration since the field is not needed at update time
  target_id_ = registerProvision(target_name, 0);

  try {
    source_size_ = getFieldSize(source_id_);
  } catch (const UndefinedFieldSizeError &) {}
  if (source_size_ > 0) init();
}

void HistoryOperator::init() {
  if (target_size_ > 0) return;
  source_size_ = getFieldSize(source_id_);
  target_size_ = source_size_ * history_len_;
  setFieldSize(target_id_, target_size_);

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
  for (const auto &frame : history_) {
    stackField(frame, offset, output_);
  }
  STEPIT_ASSERT(offset == output_.size(), "History field size ({}) does not match the target size ({}).", offset,
                output_.size());
}

bool HistoryOperator::update(FieldMap &context) {
  if (history_.empty()) {
    history_.fill(context.at(source_id_));
    updateOutput();
  }

  if (include_current_frame_) {
    push(context.at(source_id_));
    updateOutput();
  }

  context[target_id_] = output_;
  return true;
}

void HistoryOperator::commit(const FieldMap &context) {
  if (not include_current_frame_) {
    auto it = context.find(source_id_);
    STEPIT_ASSERT(it != context.end(), "Field '{}' not found at runtime.", getFieldName(source_id_));
    push(it->second);
    updateOutput();
  }
}

MaskedFillOperator::MaskedFillOperator(const yml::Node &config) {
  if (config["field"].hasValue()) {
    auto field_name = config["field"].as<std::string>();
    source_id_      = registerRequirement(field_name);
    target_id_      = source_id_;
  } else {
    STEPIT_ASSERT(config["source"].hasValue() and config["target"].hasValue(),
                  "masked_fill op must contain 'field' or both 'source' and 'target'.");
    auto source_name = config["source"].as<std::string>();
    auto target_name = config["target"].as<std::string>();
    source_id_       = registerRequirement(source_name);
    target_id_       = registerProvision(target_name, 0);
  }

  if (config["indices"].hasValue()) {
    const auto indices_node = config["indices"];
    STEPIT_ASSERT(indices_node.isSequence() and indices_node.size() > 0,
                  "'indices' in masked_fill op must be a non-empty sequence.");
    for (const auto &index_node : indices_node) {
      indices_.push_back(index_node.as<FieldSize>());
    }
  } else {
    auto start = config["start"].as<FieldSize>();
    auto end   = config["end"].as<FieldSize>();
    STEPIT_ASSERT(end > start, "Slice range [start={}, end={}) is invalid.", start, end);
    for (FieldSize i{start}; i < end; ++i) indices_.push_back(i);
  }
  config["value"].to(value_, true);

  try {
    init();
  } catch (const UndefinedFieldSizeError &) {}
}

void MaskedFillOperator::init() {
  if (field_size_ > 0) return;
  field_size_ = getFieldSize(source_id_);
  for (auto index : indices_) {
    STEPIT_ASSERT(index < field_size_, "masked_fill index {} is out of range [0, {}) for '{}'.", index, field_size_,
                  getFieldName(source_id_));
  }
  setFieldSize(target_id_, field_size_);
  buffer_.resize(field_size_);
}

bool MaskedFillOperator::update(FieldMap &context) {
  buffer_ = context.at(source_id_);
  for (auto index : indices_) {
    buffer_[index] = value_;
  }
  context[target_id_] = buffer_;
  return true;
}

SliceOperator::SliceOperator(const yml::Node &config) {
  STEPIT_ASSERT(config["source"].hasValue() and config["target"].hasValue(),
                "Slice op must contain 'source' and 'target'.");

  if (config["indices"].hasValue()) {
    const auto indices_node = config["indices"];
    STEPIT_ASSERT(indices_node.isSequence() and indices_node.size() > 0,
                  "'indices' in slice op must be a non-empty sequence.");
    for (const auto &index_node : indices_node) {
      indices_.push_back(index_node.as<FieldSize>());
    }
  } else {
    auto start = config["start"].as<FieldSize>();
    auto end   = config["end"].as<FieldSize>();
    STEPIT_ASSERT(end > start, "Slice range [start={}, end={}) is invalid.", start, end);
    for (FieldSize i{start}; i < end; ++i) indices_.push_back(i);
  }

  source_id_ = registerRequirement(config["source"].as<std::string>());
  target_id_ = registerProvision(config["target"].as<std::string>(), static_cast<FieldSize>(indices_.size()));
}

void SliceOperator::init() {
  auto source_size = getFieldSize(source_id_);
  for (auto index : indices_) {
    STEPIT_ASSERT(index < source_size, "Slice index {} is out of range [0, {}) for '{}'.", index, source_size,
                  getFieldName(source_id_));
  }
  buffer_.resize(getFieldSize(target_id_));
}

bool SliceOperator::update(FieldMap &context) {
  const auto &source = context.at(source_id_);
  for (std::size_t i{}; i < indices_.size(); ++i) {
    buffer_[static_cast<Eigen::Index>(i)] = source[indices_[i]];
  }
  context[target_id_] = buffer_;
  return true;
}

SplitOperator::SplitOperator(const yml::Node &config) {
  config.assertHasValue("source", "targets");
  source_id_ = registerRequirement(config["source"].as<std::string>());

  const auto targets_node = config["targets"];
  targets_node.assertSequence();
  for (const auto &target_node : targets_node) {
    target_node.assertMap();
    auto name = target_node["name"].as<std::string>();
    auto size = target_node["size"].as<FieldSize>();
    target_ids_.push_back(registerProvision(name, size));
    segment_sizes_.push_back(size);
  }
}

void SplitOperator::init() {
  auto source_size     = getFieldSize(source_id_);
  FieldSize total_size = std::accumulate(segment_sizes_.begin(), segment_sizes_.end(), static_cast<FieldSize>(0));
  STEPIT_ASSERT(total_size == source_size, "Split sizes ({}) do not match source size ({}) for '{}'.", total_size,
                source_size, getFieldName(source_id_));
}

bool SplitOperator::update(FieldMap &context) {
  splitFields(context.at(source_id_), target_ids_, context);
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

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
    config.require(config["source"].hasValue() and config["target"].hasValue(),
                   "Specify 'field' or both 'source' and 'target' for 'affine' operator");
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

  auto scale_node = node_["scale"];
  auto std_node   = node_["std"];
  if (scale_node.hasValue()) {
    scale_node.to(scale_);
  } else if (std_node.hasValue()) {
    ArrXf std{ArrXf::Ones(field_size_)};
    std_node.to(std);
    std_node.require((std > kEPS).all(), "'std' values must be positive");
    scale_ = std.cwiseInverse();
  }

  auto bias_node = node_["bias"];
  auto mean_node = node_["mean"];
  if (bias_node.hasValue()) {
    bias_node.to(bias_);
  } else if (mean_node.hasValue()) {
    ArrXf mean{ArrXf::Zero(field_size_)};
    mean_node.to(mean);
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
  config.throwIf(source_name == target_name, "'source' and 'target' must not be the same");
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
  config["history_len"].require(history_len_ > 0, "'history_len' must be greater than 0");
  config.to(indices_);
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
  indices_.canonicalize(history_len_);
  target_size_ = source_size_ * static_cast<FieldSize>(indices_.size());
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
  for (auto index : indices_) {
    stackField(history_[index], offset, output_);
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

void HistoryOperator::postStep(const FieldMap &context) {
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
    config.require(config["source"].hasValue() and config["target"].hasValue(),
                   "Specify 'field' or both 'source' and 'target' for 'masked_fill' operator");
    auto source_name = config["source"].as<std::string>();
    auto target_name = config["target"].as<std::string>();
    source_id_       = registerRequirement(source_name);
    target_id_       = registerProvision(target_name, 0);
  }
  config.to(indices_);
  config["value"].to(value_, true);

  try {
    init();
  } catch (const UndefinedFieldSizeError &) {}
}

void MaskedFillOperator::init() {
  if (field_size_ > 0) return;
  field_size_ = getFieldSize(source_id_);
  indices_.canonicalize(field_size_);
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
  config.require(config["source"].hasValue() and config["target"].hasValue(),
                 "Slice op must contain 'source' and 'target'");
  source_id_ = registerRequirement(config["source"].as<std::string>());
  target_id_ = registerProvision(config["target"].as<std::string>(), 0);
  config.to(indices_);

  try {
    init();
  } catch (const UndefinedFieldSizeError &) {}
}

void SliceOperator::init() {
  auto source_size = getFieldSize(source_id_);
  indices_.canonicalize(source_size);
  setFieldSize(target_id_, static_cast<FieldSize>(indices_.size()));
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

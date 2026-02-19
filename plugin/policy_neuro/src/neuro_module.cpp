#include <numeric>

#include <stepit/policy_neuro/neuro_module.h>

namespace stepit {
namespace neuro_policy {
NeuroModule::NeuroModule(const std::string &name, const std::string &home_dir)
    : config_(yml::loadFile(home_dir + "/" + name + ".yml")) {
  run_name_ = yml::readIf<std::string>(config_["run"], "name", "unknown");
  yml::setIf(config_, "assert_all_finite", assert_all_finite_);

  displayFormattedBanner(60, kGreen, "NeuroModule {} ({})", name, run_name_);
  nn_ = NnrtApi::make("", home_dir + "/" + name + ".onnx", config_);

  for (const auto &input_name : nn_->getInputNames()) {
    if (not nn_->isInputRecurrent(input_name)) input_names_.push_back(input_name);
  }
  for (const auto &output_name : nn_->getOutputNames()) {
    if (not nn_->isOutputRecurrent(output_name)) output_names_.push_back(output_name);
  }
  STEPIT_ASSERT(input_names_.size() >= 1, "The neural network must have at least one ordinary input.");
  STEPIT_ASSERT(output_names_.size() >= 1, "The neural network must have at least one ordinary output.");
  parseFields(true, input_names_, input_field_names_, input_field_sizes_, input_dims_, input_fields_);
  parseFields(false, output_names_, output_field_names_, output_field_sizes_, output_dims_, output_fields_);

  input_arr_.resize(input_names_.size());
  for (std::size_t i{}; i < input_names_.size(); ++i) {
    STEPIT_ASSERT_EQ(nn_->getInputSize(input_names_[i]), input_dims_[i], "Input dimension mismatch.");
    input_arr_[i].setZero(input_dims_[i]);
    requirements_.insert(input_fields_[i].begin(), input_fields_[i].end());
  }
  for (std::size_t i{}; i < output_names_.size(); ++i) {
    STEPIT_ASSERT_EQ(nn_->getOutputSize(output_names_[i]), output_dims_[i], "Output dimension mismatch.");
    provisions_.insert(output_fields_[i].begin(), output_fields_[i].end());
  }

  if (STEPIT_VERBOSITY >= kInfo) {
    nn_->printInfo();
    STEPIT_LOGNT("Input:");
    for (std::size_t i{}; i < input_names_.size(); ++i) {
      STEPIT_LOGNT("- {}:", input_names_[i]);
      for (auto field : input_fields_[i]) STEPIT_LOGNT("  - {} ({})", getFieldName(field), getFieldSize(field));
    }
    STEPIT_LOGNT("Output:");
    for (std::size_t i{}; i < output_names_.size(); ++i) {
      STEPIT_LOGNT("- {}:", output_names_[i]);
      for (auto field : output_fields_[i]) STEPIT_LOGNT("  - {} ({})", getFieldName(field), getFieldSize(field));
    }
  }
  nn_->warmup();
}

bool NeuroModule::reset() {
  nn_->clearState();
  return true;
}

bool NeuroModule::update(const LowState &, ControlRequests &, FieldMap &context) {
  for (std::size_t i{}; i < input_names_.size(); ++i) {
    concatFields(context, input_fields_[i], input_arr_[i]);
    if (assert_all_finite_ and not input_arr_[i].allFinite()) {
      STEPIT_CRIT("Indices '{}' of input '{}' are not all-finite.", getNonFiniteIndices(input_arr_[i]), input_names_[i]);
      return false;
    }
    nn_->setInput(input_names_[i], input_arr_[i].data());
  }
  nn_->runInference();
  for (std::size_t i{}; i < output_names_.size(); ++i) {
    cmArrXf output{nn_->getOutput(output_names_[i]), output_dims_[i]};
    if (assert_all_finite_ and not output.allFinite()) {
      STEPIT_CRIT("Indices '{}' of output '{}' are not all-finite.", getNonFiniteIndices(output), output_names_[i]);
      return false;
    }
    splitFields(output, output_fields_[i], context);
  }
  return true;
}

void NeuroModule::parseFields(bool is_input, const FieldNameVec &node_names, std::vector<FieldNameVec> &field_names,
                              std::vector<FieldSizeVec> &field_sizes, FieldSizeVec &total_dims,
                              std::vector<FieldIdVec> &fields) {
  std::string key = is_input ? yml::getDefinedKey(config_, "input_field", "inputs")
                             : yml::getDefinedKey(config_, "output_field", "outputs");
  yml::assertHasValue(config_, key);
  std::string identifier = is_input ? "input" : "output";

  const auto &fields_config = config_[key];
  std::size_t num_nodes     = node_names.size();
  STEPIT_ASSERT(fields_config.IsSequence() or fields_config.IsMap(),
                "'{}' must be a map, or a sequence only if the neural network has only one ordinary {}.", key,
                identifier);
  STEPIT_ASSERT(not fields_config.IsSequence() or num_nodes == 1,
                "'{}' must be a map if the neural network has multiple ordinary {}s.", key, identifier);
  STEPIT_ASSERT(
      not fields_config.IsMap() or num_nodes == fields_config.size(),
      "Expected '{}' to contain exactly {} entries corresponding to the neural network's ordinary {}s, but got {}.",
      key, num_nodes, identifier, fields_config.size());

  field_names.resize(num_nodes);
  field_sizes.resize(num_nodes);
  total_dims.resize(num_nodes);
  fields.resize(num_nodes);

  auto buildNodeFieldProperties = [&](const YAML::Node &field_entries, std::size_t node_index) {
    for (const auto &entry : field_entries) {
      std::string field_name;
      FieldSize field_size{};
      yml::setTo(entry, "name", field_name);
      yml::setTo(entry, "size", field_size);

      field_names[node_index].push_back(field_name);
      field_sizes[node_index].push_back(field_size);
      fields[node_index].push_back(registerField(field_name, field_size));
    }
  };
  if (fields_config.IsSequence()) {
    buildNodeFieldProperties(fields_config, 0);
  } else {
    for (std::size_t i{}; i < num_nodes; ++i) {
      const std::string &node_name = node_names[i];
      STEPIT_ASSERT(yml::hasValue(fields_config, node_name) and fields_config[node_name].IsSequence(),
                    "Expected '{}' for node '{}' to be a sequence.", key, node_name);
      buildNodeFieldProperties(fields_config[node_name], i);
    }
  }

  for (std::size_t i{}; i < num_nodes; ++i) {
    total_dims[i] = std::accumulate(field_sizes[i].begin(), field_sizes[i].end(), static_cast<FieldSize>(0));
  }
}

NeuroActor::NeuroActor(const PolicySpec &, const std::string &home_dir) : NeuroModule("actor", home_dir) {}

NeuroEstimator::NeuroEstimator(const PolicySpec &, const std::string &home_dir) : NeuroModule("estimator", home_dir) {}

STEPIT_REGISTER_MODULE(actor, kDefPriority, Module::make<NeuroActor>);
STEPIT_REGISTER_MODULE(estimator, kDefPriority, Module::make<NeuroEstimator>);
STEPIT_REGISTER_FIELD_SOURCE(action, kDefPriority, Module::make<NeuroActor>);
}  // namespace neuro_policy
}  // namespace stepit

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
  STEPIT_ASSERT(input_names_.size() >= 1, "The neural network must have at least one non-recurrent input.");
  STEPIT_ASSERT(output_names_.size() >= 1, "The neural network must have at least one non-recurrent output.");
  parseFields("input_field", input_names_, input_field_names_, input_field_dims_, input_dims_, input_fields_);
  parseFields("output_field", output_names_, output_field_names_, output_field_dims_, output_dims_, output_fields_);

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

bool NeuroModule::update(const LowState &low_state, ControlRequests &requests, FieldMap &result) {
  bool all_finite = true;
  for (std::size_t i{}; i < input_names_.size(); ++i) {
    assembleFields(result, input_fields_[i], input_arr_[i]);
    if (assert_all_finite_ and not input_arr_[i].allFinite()) {
      STEPIT_CRIT("Indices '{}' of input '{}' is not all-finite.", getNonFiniteIndices(input_arr_[i]), input_names_[i]);
      all_finite = false;
    }
    nn_->setInput(input_names_[i], input_arr_[i].data());
  }
  if (not all_finite) return false;
  nn_->runInference();
  for (std::size_t i{}; i < output_names_.size(); ++i) {
    cmArrXf output{nn_->getOutput(output_names_[i]), output_dims_[i]};
    if (assert_all_finite_ and not output.allFinite()) {
      STEPIT_CRIT("Indices '{}' of output '{}' is not all-finite.", getNonFiniteIndices(output), output_names_[i]);
      all_finite = false;
    }
    splitFields(output, output_fields_[i], result);
  }
  return all_finite;
}

FieldId NeuroModule::addField(const YAML::Node &node, std::vector<std::string> &field_names,
                              std::vector<std::uint32_t> &field_dims) {
  std::string field_name;
  std::uint32_t field_size{};
  yml::setTo(node, "name", field_name);
  yml::setTo(node, "size", field_size);

  field_names.push_back(field_name);
  field_dims.push_back(field_size);
  return registerField(field_name, field_size);
}

void NeuroModule::parseFields(const std::string &key, const std::vector<std::string> &node_names,
                              std::vector<std::vector<std::string>> &field_names,
                              std::vector<std::vector<std::uint32_t>> &field_dims,
                              std::vector<std::uint32_t> &total_dims, std::vector<std::vector<FieldId>> &fields) {
  yml::assertValid(config_, key);
  auto cfg = config_[key];

  std::size_t num_nodes = node_names.size();
  field_names.resize(num_nodes);
  field_dims.resize(num_nodes);
  total_dims.resize(num_nodes);
  fields.resize(num_nodes);

  if (cfg.IsSequence()) {
    STEPIT_ASSERT_EQ(num_nodes, 1UL, "'{}' must be a map if the neural network has multiple non-recurrent inputs.",
                     key);
    for (const auto &node : cfg) {
      fields[0].push_back(addField(node, field_names[0], field_dims[0]));
    }
  } else {
    STEPIT_ASSERT(cfg.IsMap(), "'{}' must be a map if the neural network has multiple non-recurrent inputs.", key);
    STEPIT_ASSERT_EQ(num_nodes, cfg.size(),
                     "The number of keys in '{}' must match the number of non-recurrent inputs of the neural network.",
                     key);
    for (std::size_t i{}; i < num_nodes; ++i) {
      const std::string &node_name = node_names[i];
      yml::assertValid(cfg, node_name);
      for (const auto &node : cfg[node_name]) {
        fields[i].push_back(addField(node, field_names[i], field_dims[i]));
      }
    }
  }

  for (std::size_t i{}; i < num_nodes; ++i) {
    total_dims[i] = std::accumulate(field_dims[i].begin(), field_dims[i].end(), 0U);
  }
}

NeuroActor::NeuroActor(const PolicySpec &, const std::string &home_dir) : NeuroModule("actor", home_dir) {}

NeuroEstimator::NeuroEstimator(const PolicySpec &, const std::string &home_dir) : NeuroModule("estimator", home_dir) {}

STEPIT_REGISTER_MODULE(actor, kDefPriority, Module::make<NeuroActor>);
STEPIT_REGISTER_MODULE(estimator, kDefPriority, Module::make<NeuroEstimator>);
STEPIT_REGISTER_FIELD_SOURCE(action, kDefPriority, Module::make<NeuroActor>);
}  // namespace neuro_policy
}  // namespace stepit

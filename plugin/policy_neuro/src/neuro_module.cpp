#include <numeric>

#include <stepit/policy_neuro/neuro_module.h>

namespace stepit {
namespace neuro_policy {
NeuroModule::NeuroModule(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : Module(policy_spec, ModuleSpec(module_spec, "neuro_module")) {
  config_["nnrt_factory"].to(nnrt_factory_, true);
  model_path_ = config_["model_path"].as<std::string>(name_);
  STEPIT_ASSERT(not model_path_.empty(), "'model_path' cannot be empty.");
  model_path_ = model_path_[0] == '/' ? model_path_ : joinPaths(policy_spec.home_dir, model_path_);
  config_["run"]["name"].to(run_name_, true);
  config_["assert_all_finite"].to(assert_all_finite_, true);

  std::string displayed_name = run_name_.empty() ? name_ : fmt::format("{} ({})", name_, run_name_);
  displayFormattedBanner(60, kGreen, "NeuroModule {}", displayed_name);
  nn_ = NnrtApi::make(nnrt_factory_, model_path_, config_);
  if (STEPIT_VERBOSITY >= kInfo) nn_->printInfo();

  for (const auto &input_name : nn_->getInputNames()) {
    if (not nn_->isInputRecurrent(input_name)) {
      input_names_.push_back(input_name);
      std::size_t input_size = nn_->getInputSize(input_name);
      input_dims_.push_back(input_size);
      input_arr_.push_back(ArrXf::Zero(input_size));
    }
  }
  for (const auto &output_name : nn_->getOutputNames()) {
    if (not nn_->isOutputRecurrent(output_name)) {
      output_names_.push_back(output_name);
      output_dims_.push_back(nn_->getOutputSize(output_name));
    }
  }
  STEPIT_ASSERT(input_names_.size() >= 1, "The neural network must have at least one ordinary input.");
  STEPIT_ASSERT(output_names_.size() >= 1, "The neural network must have at least one ordinary output.");
  parseFields(true, input_names_, input_dims_, input_field_names_, input_field_sizes_, input_field_ids_);
  parseFields(false, output_names_, output_dims_, output_field_names_, output_field_sizes_, output_field_ids_);

  if (STEPIT_VERBOSITY >= kInfo) {
    STEPIT_LOGNT("Input:");
    printNodeFields(input_names_, input_field_ids_);
    STEPIT_LOGNT("Output:");
    printNodeFields(output_names_, output_field_ids_);
  }
  nn_->warmup();
}

bool NeuroModule::reset() {
  nn_->clearState();
  return true;
}

bool NeuroModule::update(const LowState &, ControlRequests &, FieldMap &context) {
  for (std::size_t i{}; i < input_names_.size(); ++i) {
    concatFields(context, input_field_ids_[i], input_arr_[i]);
    if (assert_all_finite_ and not input_arr_[i].allFinite()) {
      STEPIT_CRIT("Indices '{}' of input '{}' are not all-finite.", getNonFiniteIndices(input_arr_[i]),
                  input_names_[i]);
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
    splitFields(output, output_field_ids_[i], context);
  }
  return true;
}

void NeuroModule::parseFields(bool is_input, const FieldNameVec &node_names, const FieldSizeVec &node_sizes,
                              std::vector<FieldNameVec> &field_names, std::vector<FieldSizeVec> &field_sizes,
                              std::vector<FieldIdVec> &field_ids) {
  const std::string identifier = is_input ? "input" : "output";
  const std::string fields_key = is_input ? config_.getDefinedKey({"input_field", "inputs", "input_fields"})
                                          : config_.getDefinedKey({"output_field", "outputs", "output_fields"});
  const std::size_t num_nodes  = node_names.size();
  field_names.resize(num_nodes);
  field_sizes.resize(num_nodes);
  field_ids.resize(num_nodes);

  if (fields_key.empty()) {
    for (std::size_t i{}; i < num_nodes; ++i) {
      const auto &field_name = node_names[i];
      FieldSize field_size   = node_sizes[i];
      FieldId field_id       = registerField(field_name, field_size);
      field_names[i].push_back(field_name);
      field_sizes[i].push_back(field_size);
      field_ids[i].push_back(field_id);
      if (is_input) {
        registerRequirement(field_id);
      } else {
        STEPIT_ASSERT(provisions().count(field_id) == 0, "Field '{}' is already registered as a provision.",
                      getFieldName(field_id));
        registerProvision(field_id);
      }
    }
    return;
  }

  auto buildNodeFieldProperties = [&](const yml::Node &field_entries, std::size_t node_index) {
    for (const auto &entry : field_entries) {
      std::string field_name;
      FieldSize field_size{};
      entry["name"].to(field_name);
      entry["size"].to(field_size);

      field_names[node_index].push_back(field_name);
      field_sizes[node_index].push_back(field_size);
      field_ids[node_index].push_back(registerField(field_name, field_size));
    }
  };

  const auto &fields_config = config_[fields_key];
  STEPIT_ASSERT(fields_config.isSequence() or fields_config.isMap(),
                "If defined, '{}' must be a map, or a sequence only if the {} has only one ordinary {}.", fields_key,
                name_, identifier);
  if (fields_config.isSequence()) {
    STEPIT_ASSERT(num_nodes == 1, "'{}' should not be a sequence if the {} has multiple ordinary {}s.", fields_key,
                  name_, identifier);
    buildNodeFieldProperties(fields_config, 0);
  } else {
    for (std::size_t i{}; i < num_nodes; ++i) {
      const std::string &node_name   = node_names[i];
      const auto &node_field_entries = fields_config[node_name];
      node_field_entries.assertIterable();
      if (node_field_entries.isSequence()) {
        buildNodeFieldProperties(fields_config[node_name], i);
      } else {
        std::string field_name = node_name;
        FieldSize field_size   = node_sizes[i];
        node_field_entries["name"].to(field_name, true);
        node_field_entries["size"].to(field_size, true);
        field_names[i].push_back(field_name);
        field_sizes[i].push_back(field_size);
        field_ids[i].push_back(registerField(field_name, field_size));
      }
    }
  }

  for (std::size_t i{}; i < num_nodes; ++i) {
    std::size_t total_size = std::accumulate(field_sizes[i].begin(), field_sizes[i].end(), static_cast<FieldSize>(0));
    STEPIT_ASSERT(total_size == node_sizes[i],
                  "Total size of fields for node '{}' must equal the neural network's {} size {}, but got {}.",
                  node_names[i], identifier, node_sizes[i], total_size);
    for (auto field_id : field_ids[i]) {
      if (is_input) {
        registerRequirement(field_id);
      } else {
        STEPIT_ASSERT(provisions().count(field_id) == 0, "Field '{}' is already registered as a provision.",
                      getFieldName(field_id));
        registerProvision(field_id);
      }
    }
  }
}

void NeuroModule::printNodeFields(const std::vector<std::string> &node_names,
                                  const std::vector<FieldIdVec> &field_ids) {
  for (std::size_t i{}; i < node_names.size(); ++i) {
    if (field_ids[i].size() == 1) {
      if (getFieldName(field_ids[i][0]) == node_names[i]) {
        STEPIT_LOGNT("- {} ({})", node_names[i], getFieldSize(field_ids[i][0]));
      } else {
        STEPIT_LOGNT("- {}: {} ({})", node_names[i], getFieldName(field_ids[i][0]), getFieldSize(field_ids[i][0]));
      }
      continue;
    }
    STEPIT_LOGNT("- {}:", node_names[i]);
    for (auto field : field_ids[i]) STEPIT_LOGNT("  - {} ({})", getFieldName(field), getFieldSize(field));
  }
}

NeuroActor::NeuroActor(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : NeuroModule(policy_spec, ModuleSpec(module_spec, "actor")) {}

NeuroEstimator::NeuroEstimator(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : NeuroModule(policy_spec, ModuleSpec(module_spec, "estimator")) {}

STEPIT_REGISTER_MODULE(neuro, kDefPriority, Module::make<NeuroModule>);
STEPIT_REGISTER_MODULE(actor, kDefPriority, Module::make<NeuroActor>);
STEPIT_REGISTER_MODULE(estimator, kDefPriority, Module::make<NeuroEstimator>);
STEPIT_REGISTER_FIELD_SOURCE(action, kDefPriority, Module::make<NeuroActor>);
}  // namespace neuro_policy
}  // namespace stepit

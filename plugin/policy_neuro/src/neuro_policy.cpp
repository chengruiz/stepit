#include <stepit/policy_neuro/neuro_policy.h>

namespace stepit {
namespace neuro_policy {
NeuroPolicy::NeuroPolicy(const RobotSpec &robot_spec, const std::string &home_dir)
    : spec_(robot_spec, home_dir), config_(yml::loadFile(joinPaths(home_dir, "policy.yml"))) {
  config_["control_freq"].to(spec_.control_freq);
  spec_.policy_config = config_;
  spec_.policy_name   = config_["name"].as<std::string>("");
  spec_.trusted       = config_["trusted"].as<bool>(false);
  tailored_           = config_["tailored"].as<std::string>("");

  std::string default_action_key = config_.getDefinedKey({"default_action", "action_mean"});
  config_[default_action_key].to(spec_.default_action, true);
  STEPIT_ASSERT(tailored_.empty() or tailored_ == robot_spec.robot_name,
                "Policy tailored for '{}' cannot be used with robot '{}'.", tailored_, robot_spec.robot_name);
  action_id_ = registerField("action", 0);
  displayFormattedBanner(60, kGreen, "NeuroPolicy {} ({}Hz)", spec_.policy_name, getControlFreq());

  // Add modules read from the YAML file
  std::string module_key = config_.getDefinedKey({"field_source", "module", "modules"});
  auto modules_config    = config_[module_key];
  if (modules_config.hasValue()) {
    modules_config.assertSequence();
    for (const auto &module_node : modules_config) {
      std::string factory_name = module_node.as<std::string>();
      std::string module_name  = "";

      auto at_pos = factory_name.find("@");
      if (at_pos != std::string::npos) {
        module_name  = factory_name.substr(0, at_pos);
        factory_name = factory_name.substr(at_pos + 1);
      }
      addModule(Module::make(factory_name, spec_, module_name), false);
    }
  }

  // Add the actuator and the actor
  std::string actuator_type = "position";
  if (config_["actuator"].hasValue()) config_["actuator"]["type"].to(actuator_type, true);
  auto actuator = Actuator::make(actuator_type, spec_, "");
  actuator_     = actuator.get();
  addModule(std::move(actuator), true);
  if (available_fields_.find(action_id_) == available_fields_.end() and
      unavailable_fields_.find(action_id_) == unavailable_fields_.end() and
      unresolved_fields_.find(action_id_) == unresolved_fields_.end()) {
    addModule(makeFieldSource("action", spec_, ""), false);
  }

  // Automatically resolve field dependencies
  while (not unresolved_modules_.empty()) {
    FieldId requirement{};
    for (auto field : unresolved_modules_.front()->requirements()) {
      if (available_fields_.find(field) != available_fields_.end()) continue;
      if (unresolved_fields_.find(field) != unresolved_fields_.end()) {
        STEPIT_THROW("Find circular dependency '{}' in module '{}'.", getFieldName(field),
                     getTypeName(*unresolved_modules_.front()));
      }
      requirement = field;
      break;
    }
    addModule(makeFieldSource(getFieldName(requirement), spec_, ""), true);
  }
  STEPIT_ASSERT(unavailable_fields_.empty() and unresolved_modules_.empty(), "Policy is not fully resolved.");

  for (const auto &module : resolved_modules_) module->init();
  auto action_dim = getFieldSize(action_id_);
  action_.setZero(action_dim);

  publish_fields_               = STEPIT_VERBOSITY <= kDbug;  // default value
  yml::Node publish_fields_node = config_["publish_fields"];
  if (publish_fields_node.hasValue()) {
    if (publish_fields_node.isBool()) {
      publish_fields_ = publish_fields_node.as<bool>();
    } else {
      STEPIT_ASSERT(publish_fields_node.isSequence(), "'publish_fields' must be a boolean or a sequence.");
      publish_fields_ = true;
      for (const auto &field_name : publish_fields_node) {
        published_fields_.insert(getFieldId(field_name.as<std::string>()));
      }
    }
  }

  STEPIT_DBUGNT("Modules:");
  for (const auto &module : resolved_modules_) {
    STEPIT_DBUGNT("- {} ({})", module->name(), getTypeName(*module));
  }
  displayFormattedBanner(60);
}

void NeuroPolicy::addModule(Module::Ptr module, bool first) {
  // Add the provisions to unresolved_fields_
  for (auto field : module->provisions()) {
    if (available_fields_.find(field) != available_fields_.end()) {
      STEPIT_THROW("Multiple sources for field '{}'.", getFieldName(field));
    }
    unresolved_fields_.insert(field);
    unavailable_fields_.erase(field);
  }
  // Add the unavailable requirements to unavailable_fields_
  for (auto field : module->requirements()) {
    if (available_fields_.find(field) == available_fields_.end()) {
      unavailable_fields_.insert(field);
    }
  }
  // Add the module to unresolved_modules_
  if (first) {
    unresolved_modules_.push_front(std::move(module));
  } else {
    unresolved_modules_.push_back(std::move(module));
  }

  // Check if the module is already resolved
  while (not unresolved_modules_.empty()) {
    const auto &front_module = unresolved_modules_.front();
    if (not isSatisfied(front_module->requirements())) break;
    for (auto field : front_module->provisions()) {
      available_fields_.insert(field);
      unresolved_fields_.erase(field);
      unavailable_fields_.erase(field);
    }
    resolved_modules_.push_back(std::move(unresolved_modules_.front()));
    unresolved_modules_.pop_front();
  }
}

bool NeuroPolicy::reset() {
  for (const auto &module : resolved_modules_) {
    if (not module->reset()) {
      STEPIT_CRIT("Failed to initialize '{}'.", getTypeName(*module));
      return false;
    }
  }

  num_steps_ = 0;
  context_.clear();
  return true;
}

bool NeuroPolicy::act(const LowState &low_state, ControlRequests &requests, LowCmd &cmd) {
  context_.clear();
  for (const auto &module : resolved_modules_) {
    if (not module->update(low_state, requests, context_)) {
      STEPIT_CRIT("Failed to update '{}'.", getTypeName(*module));
      context_.clear();
      return false;
    }
  }
  action_ = context_.at(action_id_);
  actuator_->setLowCmd(cmd, action_);

  ++num_steps_;
  return true;
}

void NeuroPolicy::postAct() {
  for (const auto &module : resolved_modules_) {
    module->postStep(context_);
  }

  if (publish_fields_) {
    for (const auto &it : context_) {
      if (published_fields_.empty() or published_fields_.find(it.first) != published_fields_.end()) {
        publisher::publishArray("field/" + getFieldName(it.first), it.second);
      }
    }
  }
}

void NeuroPolicy::exit() {
  context_.clear();
  for (const auto &module : resolved_modules_) module->exit();
}

bool NeuroPolicy::isSatisfied(const std::set<FieldId> &requirements) const {
  return std::all_of(requirements.begin(), requirements.end(),
                     [this](FieldId field) { return available_fields_.find(field) != available_fields_.end(); });
}

STEPIT_REGISTER_POLICY(neuro, kDefPriority, Policy::make<NeuroPolicy>);
}  // namespace neuro_policy
}  // namespace stepit

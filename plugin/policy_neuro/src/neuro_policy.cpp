#include <stepit/policy_neuro/neuro_policy.h>

namespace stepit {
namespace neuro_policy {
NeuroPolicy::NeuroPolicy(const RobotSpec &robot_spec, const std::string &home_dir)
    : Policy(robot_spec), config_(yml::loadFile(home_dir + "/policy.yml")) {
  yml::setTo(config_, "control_freq", spec_.control_freq);
  yml::setIf(config_, "name", spec_.policy_name);
  yml::setIf(config_, "trusted", spec_.trusted);
  yml::setIf(config_, "tailored", tailored_);
  STEPIT_ASSERT(tailored_.empty() or tailored_ == robot_spec.robot_name,
                "Policy tailored for '{}' cannot be used with robot '{}'.", tailored_, robot_spec.robot_name);
  action_id_ = registerField("action", 0);
  displayFormattedBanner(60, kGreen, "NeuroPolicy {} ({}Hz)", spec_.policy_name, getControlFreq());

  // Add field sources read from the YAML file
  auto module_node = config_["module"] ? config_["module"] : config_["field_source"];  // backward compatibility
  if (module_node) {
    STEPIT_ASSERT(module_node.IsSequence(), "'module' must be a sequence.");
    for (const auto &module : module_node) {
      addModule(Module::make(module.as<std::string>(), spec_, home_dir), false);
    }
  }
  // Add the actuator and the actor
  std::string actuator_type = "position";
  if (config_["actuator"]) yml::setIf(config_["actuator"], "type", actuator_type);
  auto actuator = Actuator::make(actuator_type, spec_, home_dir);
  actuator_     = actuator.get();
  addModule(std::move(actuator), true);
  if (available_fields_.find(action_id_) == available_fields_.end() and
      unavailable_fields_.find(action_id_) == unavailable_fields_.end() and
      unresolved_fields_.find(action_id_) == unresolved_fields_.end()) {
    addModule(makeFieldSource("action", spec_, home_dir), false);
  }
  // Automatically resolve field dependencies
  while (not unresolved_modules_.empty()) {
    FieldId requirement{};
    for (auto field : unresolved_modules_.front()->requirements()) {
      if (available_fields_.find(field) != available_fields_.end()) continue;
      if (unresolved_fields_.find(field) != unresolved_fields_.end()) {
        STEPIT_ERROR("Find circular dependency '{}' in module '{}'.", getFieldName(field),
                     getTypeName(*unresolved_modules_.front()));
      }
      requirement = field;
      break;
    }
    addModule(makeFieldSource(getFieldName(requirement), spec_, home_dir), true);
  }
  STEPIT_ASSERT(unavailable_fields_.empty() and unresolved_modules_.empty(), "Policy is not fully resolved.");

  auto action_dim = getFieldSize(action_id_);
  action_.setZero(action_dim);

  publish_fields_                = STEPIT_VERBOSITY <= kDbug;  // default value
  YAML::Node publish_fields_node = config_["publish_fields"];
  if (publish_fields_node) {
    if (yml::isBool(publish_fields_node)) {
      publish_fields_ = publish_fields_node.as<bool>();
    } else {
      STEPIT_ASSERT(publish_fields_node.IsSequence(), "'publish_fields' must be a boolean or a sequence.");
      publish_fields_ = true;
      for (const auto &field_name : publish_fields_node) {
        published_fields_.insert(getFieldId(yml::readAs<std::string>(field_name)));
      }
    }
  }

  STEPIT_DBUGNT("Field sources:");
  for (const auto &module : resolved_modules_) {
    module->initFieldProperties();
    STEPIT_DBUGNT("- {}", getTypeName(*module));
  }
  displayFormattedBanner(60);
}

void NeuroPolicy::addModule(Module::Ptr module, bool first) {
  // Add the provisions to unresolved_fields_
  for (auto field : module->provisions()) {
    if (available_fields_.find(field) != available_fields_.end()) {
      STEPIT_ERROR("Multiple sources for field '{}'.", getFieldName(field));
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
  return true;
}

bool NeuroPolicy::act(const LowState &low_state, ControlRequests &requests, LowCmd &cmd) {
  FieldMap field_map;
  for (const auto &module : resolved_modules_) {
    if (not module->update(low_state, requests, field_map)) {
      STEPIT_CRIT("Failed to update '{}'.", getTypeName(*module));
      return false;
    }
  }
  for (const auto &module : resolved_modules_) module->postUpdate(field_map);
  action_ = field_map.at(action_id_);
  actuator_->setLowCmd(cmd, action_);

  if (publish_fields_) {
    for (const auto &it : field_map) {
      if (published_fields_.empty() or published_fields_.find(it.first) != published_fields_.end()) {
        publisher::publishArray("field/" + getFieldName(it.first), it.second);
      }
    }
  }
  ++num_steps_;
  return true;
}

void NeuroPolicy::exit() {
  for (const auto &module : resolved_modules_) module->exit();
}

bool NeuroPolicy::isSatisfied(const std::set<FieldId> &requirements) const {
  return std::all_of(requirements.begin(), requirements.end(),
                     [this](FieldId field) { return available_fields_.find(field) != available_fields_.end(); });
}

STEPIT_REGISTER_POLICY(neuro, kDefPriority, Policy::make<NeuroPolicy>);
}  // namespace neuro_policy
}  // namespace stepit

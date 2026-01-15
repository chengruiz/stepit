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
  auto fs_node = config_["field_source"];
  if (fs_node) {
    STEPIT_ASSERT(fs_node.IsSequence(), "'field_source' must be a sequence.");
    for (const auto &fs : fs_node) {
      addFieldSource(FieldSource::make(fs.as<std::string>(), spec_, home_dir), false);
    }
  }
  // Add the actuator and the actor
  std::string actuator_type = "position";
  if (config_["actuator"]) yml::setIf(config_["actuator"], "type", actuator_type);
  auto actuator = Actuator::make(actuator_type, spec_, home_dir);
  actuator_     = actuator.get();
  addFieldSource(std::move(actuator), true);
  if (available_fields_.find(action_id_) == available_fields_.end() and
      unavailable_fields_.find(action_id_) == unavailable_fields_.end() and
      unresolved_fields_.find(action_id_) == unresolved_fields_.end()) {
    addFieldSource(makeSourceOfField("action", spec_, home_dir), false);
  }
  // Automatically resolve field dependencies
  while (not unresolved_fs_.empty()) {
    FieldId requirement{};
    for (auto field : unresolved_fs_.front()->requirements()) {
      if (available_fields_.find(field) != available_fields_.end()) continue;
      if (unresolved_fields_.find(field) != unresolved_fields_.end()) {
        STEPIT_ERROR("Find circular dependency '{}' in field source '{}'.", getFieldName(field),
                     getTypeName(*unresolved_fs_.front()));
      }
      requirement = field;
      break;
    }
    addFieldSource(makeSourceOfField(getFieldName(requirement), spec_, home_dir), true);
  }
  STEPIT_ASSERT(unavailable_fields_.empty() and unresolved_fs_.empty(), "Policy is not fully resolved.");

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
  for (const auto &fs : resolved_fs_) {
    fs->initFieldProperties();
    STEPIT_DBUGNT("- {}", getTypeName(*fs));
  }
  displayFormattedBanner(60);
}

void NeuroPolicy::addFieldSource(std::unique_ptr<FieldSource> fs, bool first) {
  // Add the provisions to unresolved_fields_
  for (auto field : fs->provisions()) {
    if (available_fields_.find(field) != available_fields_.end()) {
      STEPIT_ERROR("Multiple sources for field '{}'.", getFieldName(field));
    }
    unresolved_fields_.insert(field);
    unavailable_fields_.erase(field);
  }
  // Add the unavailable requirements to unavailable_fields_
  for (auto field : fs->requirements()) {
    if (available_fields_.find(field) == available_fields_.end()) {
      unavailable_fields_.insert(field);
    }
  }
  // Add the field source to unresolved_fs_
  if (first) {
    unresolved_fs_.push_front(std::move(fs));
  } else {
    unresolved_fs_.push_back(std::move(fs));
  }

  // Check if the field source is already resolved
  while (not unresolved_fs_.empty()) {
    const auto &front_fs = unresolved_fs_.front();
    if (not isSatisfied(front_fs->requirements())) break;
    for (auto field : front_fs->provisions()) {
      available_fields_.insert(field);
      unresolved_fields_.erase(field);
      unavailable_fields_.erase(field);
    }
    resolved_fs_.push_back(std::move(unresolved_fs_.front()));
    unresolved_fs_.pop_front();
  }
}

bool NeuroPolicy::reset() {
  for (const auto &fs : resolved_fs_) {
    if (not fs->reset()) {
      STEPIT_CRIT("Failed to initialize '{}'.", getTypeName(*fs));
      return false;
    }
  }

  num_steps_ = 0;
  return true;
}

bool NeuroPolicy::act(const LowState &low_state, ControlRequests &requests, LowCmd &cmd) {
  FieldMap field_map;
  for (const auto &fs : resolved_fs_) {
    if (not fs->update(low_state, requests, field_map)) {
      STEPIT_CRIT("Failed to update '{}'.", getTypeName(*fs));
      return false;
    }
  }
  for (const auto &fs : resolved_fs_) fs->postUpdate(field_map);
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
  for (const auto &fs : resolved_fs_) fs->exit();
}

bool NeuroPolicy::isSatisfied(const std::set<FieldId> &requirements) const {
  return std::all_of(requirements.begin(), requirements.end(),
                     [this](FieldId field) { return available_fields_.find(field) != available_fields_.end(); });
}

STEPIT_REGISTER_POLICY(neuro, kDefPriority, Policy::make<NeuroPolicy>);
}  // namespace neuro_policy
}  // namespace stepit

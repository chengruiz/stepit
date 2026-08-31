#include <stepit/modular_policy/module.h>

namespace stepit {
namespace modular_policy {
namespace {
std::string getSubstrBeforeSlash(const std::string &input) {
  const auto separator_pos = input.find('/');
  if (separator_pos != std::string::npos and separator_pos > 0) {
    return input.substr(0, separator_pos);
  }
  return input;
}
}  // namespace

Module::Module(const ModularPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : name_(module_spec.name), config_path_(getSubstrBeforeSlash(name_) + ".yml"), config_(module_spec.config) {
  STEPIT_ASSERT(not name_.empty(), "Module name should not be empty.");
  config_path_ = joinPaths(policy_spec.home_dir, config_path_);
  if (not config_.hasValue()) {
    if (fs::exists(config_path_)) {
      STEPIT_DBUGNT("Loading configuration for module '{}' from '{}'.", name_, config_path_);
      config_ = yml::loadFile(config_path_);
    } else {
      STEPIT_DBUGNT("No configuration found for module '{}' at '{}'. Using empty configuration.", name_, config_path_);
    }
  }
}

FieldSourceRegistry &FieldSourceRegistry::instance() {
  static FieldSourceRegistry instance;
  return instance;
}
}  // namespace modular_policy
}  // namespace stepit

#include <stepit/policy_neuro/module.h>

namespace stepit {
namespace neuro_policy {
namespace {
std::string getSubstrBeforeSlash(const std::string &input) {
  const auto separator_pos = input.find('/');
  if (separator_pos != std::string::npos and separator_pos > 0) {
    return input.substr(0, separator_pos);
  }
  return input;
}
}  // namespace

Module::Module(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : name_(module_spec.name), config_filename_(getSubstrBeforeSlash(name_) + ".yml"), config_(module_spec.config) {
  STEPIT_ASSERT(not name_.empty(), "Module name should not be empty.");
  std::string config_path = joinPaths(policy_spec.home_dir, config_filename_);
  if (not config_.hasValue()) {
    if (fs::exists(config_path)) {
      STEPIT_DBUGNT("Loading configuration for module '{}' from '{}'.", name_, config_path);
      config_ = yml::loadFile(config_path);
    } else {
      STEPIT_DBUGNT("No configuration found for module '{}' at '{}'. Using empty configuration.", name_, config_path);
    }
  }
}

FieldSourceRegistry &FieldSourceRegistry::instance() {
  static FieldSourceRegistry instance;
  return instance;
}
}  // namespace neuro_policy
}  // namespace stepit

#include <stepit/policy_neuro/field_assembler.h>

namespace stepit {
namespace neuro_policy {
FieldAssembler::FieldAssembler(const PolicySpec &, const std::string &home_dir)
    : config_(yml::loadFile(home_dir + "/field_assembler.yml")) {
  STEPIT_ASSERT(config_.IsMap(), "'field_assembler.yml' must contain a map of field assembler configurations.");

  for (const auto &node : config_) {
    std::string target_field_name;
    FieldIdVec component_ids;

    yml::setTo(node.first, target_field_name);
    STEPIT_ASSERT(node.second.IsSequence(), "Definition for '{}' must be a sequence.", target_field_name);
    for (const auto &item : node.second) {
      auto field_id = registerField(yml::readAs<std::string>(item), 0);
      if (provisions_.find(field_id) == provisions_.end()) registerRequirement(field_id);
      component_ids.push_back(field_id);
    }

    target_ids_.push_back(registerProvision(target_field_name, 0));
    component_ids_.push_back(std::move(component_ids));
  }
}

void FieldAssembler::initFieldProperties() {
  target_sizes_.resize(target_ids_.size());
  buffers_.resize(target_ids_.size());

  for (std::size_t i{}; i < target_ids_.size(); ++i) {
    std::uint32_t total_size = 0;
    for (auto component_id : component_ids_[i]) {
      auto component_size = getFieldSize(component_id);
      STEPIT_ASSERT(component_size > 0, "Size of '{}' is undefined.", getFieldName(component_id));
      total_size += component_size;
    }
    target_sizes_[i] = total_size;
    buffers_[i].resize(total_size);
    setFieldSize(target_ids_[i], total_size);
  }
}

bool FieldAssembler::update(const LowState &, ControlRequests &, FieldMap &result) {
  for (std::size_t i{}; i < target_ids_.size(); ++i) {
    assembleFields(result, component_ids_[i], buffers_[i]);
    result[target_ids_[i]] = buffers_[i];
  }
  return true;
}

STEPIT_REGISTER_MODULE(field_assembler, kDefPriority, Module::make<FieldAssembler>);
}  // namespace neuro_policy
}  // namespace stepit

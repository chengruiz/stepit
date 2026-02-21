#include <utility>

#include <stepit/policy_neuro/const_field_source.h>

namespace stepit {
namespace neuro_policy {
ConstFieldSource::ConstFieldSource(const NeuroPolicySpec &policy_spec, const std::string &name)
    : Module(nonEmptyOr(name, "const_field")), config_(loadConfig(policy_spec)) {
  STEPIT_ASSERT(config_.IsMap(), "'const_field.yml' must contain a map of constant fields.");

  for (const auto &node : config_) {
    ConstField field;
    yml::setTo(node.first, field.name);

    const auto &definition = node.second;
    STEPIT_ASSERT(definition.IsSequence() or definition.IsMap(),
                  "Constant field '{}' must be defined as a sequence or a mapping.", field.name);
    if (definition.IsSequence()) {
      yml::setTo(definition, field.value);
      field.size = static_cast<FieldSize>(field.value.size());
    } else {  // Mapping
      STEPIT_ASSERT(yml::hasValue(definition, "value"), "Constant field '{}' must contain key 'value'.", field.name);
      YAML::Node value_node = definition["value"];
      STEPIT_ASSERT(value_node.IsScalar() or value_node.IsSequence(),
                    "Constant field '{}': 'value' must be a scalar or sequence.", field.name);
      if (value_node.IsScalar()) {
        yml::setTo(definition, "size", field.size);
        field.value.setZero(field.size);
        yml::setTo(value_node, field.value);
      } else {  // Sequence
        yml::setTo(value_node, field.value);
        yml::setIf(definition, "size", field.size);
        if (field.size > 0) {
          STEPIT_ASSERT(field.value.size() == field.size,
                        "Constant field '{}' has mismatched 'size' and 'value' lengths.", field.name);
        } else {
          field.size = static_cast<FieldSize>(field.value.size());
        }
      }
    }
    field.id = registerProvision(field.name, field.size);
    fields_.push_back(std::move(field));
  }
}

bool ConstFieldSource::update(const LowState &, ControlRequests &, FieldMap &context) {
  for (const auto &field : fields_) {
    context[field.id] = field.value;
  }
  return true;
}

STEPIT_REGISTER_MODULE(const_field, kDefPriority, Module::make<ConstFieldSource>);
}  // namespace neuro_policy
}  // namespace stepit

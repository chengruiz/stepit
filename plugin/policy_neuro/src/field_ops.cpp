#include <stepit/policy_neuro/field_ops.h>

namespace stepit {
namespace neuro_policy {
FieldOps::FieldOps(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : Module(policy_spec, ModuleSpec(module_spec, "field_ops")) {
  auto operators_node = config_["operators"];
  operators_node.assertSequence();

  for (const auto &operator_node : operators_node) {
    operator_node.assertHasValue("type");
    auto type      = operator_node["type"].as<std::string>();
    auto operation = field::Operator::make(type, operator_node);

    for (auto field_id : operation->requirements()) registerRequirement(field_id);
    for (auto field_id : operation->provisions()) registerProvision(field_id);
    operations_.push_back(std::move(operation));
  }
}

void FieldOps::init() {
  for (auto &operation : operations_) {
    operation->init();
  }
}

bool FieldOps::reset() {
  for (auto &operation : operations_) {
    if (not operation->reset()) return false;
  }
  return true;
}

bool FieldOps::update(const LowState &, ControlRequests &, FieldMap &context) {
  for (auto &operation : operations_) {
    if (not operation->update(context)) return false;
  }
  return true;
}

void FieldOps::commit(const FieldMap &context) {
  for (auto &operation : operations_) {
    operation->commit(context);
  }
}

STEPIT_REGISTER_MODULE(field_ops, kDefPriority, Module::make<FieldOps>);
}  // namespace neuro_policy
}  // namespace stepit

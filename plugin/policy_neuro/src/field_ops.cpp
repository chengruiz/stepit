#include <algorithm>
#include <exception>

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
  operation_fields_resolved_.resize(operations_.size(), false);
}

bool FieldOps::init() {
  bool progressed = false;
  std::exception_ptr first_error;

  for (std::size_t i{}; i < operations_.size(); ++i) {
    if (operation_fields_resolved_[i]) continue;

    try {
      operations_[i]->init();
      operation_fields_resolved_[i] = true;
      progressed                    = true;
    } catch (const UndefinedFieldSpecError &) {
      if (not first_error) first_error = std::current_exception();
    }
  }

  if (std::all_of(operation_fields_resolved_.begin(), operation_fields_resolved_.end(),
                  [](bool resolved) { return resolved; })) {
    return true;
  }
  if (progressed) return false;

  std::rethrow_exception(first_error);
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

void FieldOps::postStep(const FieldMap &context) {
  for (auto &operation : operations_) {
    operation->postStep(context);
  }
}

STEPIT_REGISTER_MODULE(field_ops, kDefPriority, Module::make<FieldOps>);
}  // namespace neuro_policy
}  // namespace stepit

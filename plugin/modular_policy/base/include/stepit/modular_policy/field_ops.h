#ifndef STEPIT_MODULAR_POLICY_FIELD_OPS_H_
#define STEPIT_MODULAR_POLICY_FIELD_OPS_H_

#include <stepit/field/operator.h>
#include <stepit/modular_policy/module.h>

namespace stepit {
namespace modular_policy {
class FieldOps : public Module {
 public:
  FieldOps(const ModularPolicySpec &policy_spec, const ModuleSpec &module_spec);
  bool init() override;
  bool reset() override;
  bool update(const LowState &, ControlRequests &, FieldMap &context) override;
  void postStep(const FieldMap &context) override;

 private:
  std::vector<field::Operator::Ptr> operations_;
  std::vector<bool> operation_fields_resolved_;
};
}  // namespace modular_policy
}  // namespace stepit

#endif  // STEPIT_MODULAR_POLICY_FIELD_OPS_H_

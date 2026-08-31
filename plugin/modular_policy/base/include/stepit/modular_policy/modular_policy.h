#ifndef STEPIT_MODULAR_POLICY_MODULAR_POLICY_H_
#define STEPIT_MODULAR_POLICY_MODULAR_POLICY_H_

#include <list>
#include <set>
#include <vector>

#include <stepit/policy.h>
#include <stepit/utils.h>
#include <stepit/modular_policy/actuator.h>
#include <stepit/modular_policy/module.h>

namespace stepit {
namespace modular_policy {
class ModularPolicy : public Policy {
 public:
  ModularPolicy(const RobotSpec &robot_spec, const std::string &home_dir);
  const PolicySpec &getSpec() const override { return spec_; }
  bool reset() override;
  bool act(const LowState &low_state, ControlRequests &requests, LowCmd &cmd) override;
  void postAct() override;
  void exit() override;

 private:
  void addModule(Module::Ptr module, bool first);
  bool isSatisfied(const std::set<FieldId> &requirements) const;

  ModularPolicySpec spec_;
  yml::Node config_;
  std::string tailored_;
  Actuator *actuator_{nullptr};
  bool publish_fields_{false};
  std::set<FieldId> published_fields_;
  FieldId action_id_{};

  std::vector<Module::Ptr> resolved_modules_;
  std::list<Module::Ptr> unresolved_modules_;
  std::set<FieldId> available_fields_;
  std::set<FieldId> unavailable_fields_;
  std::set<FieldId> unresolved_fields_;

  std::size_t num_steps_{0};
  FieldMap context_;
  ArrXf action_;
};
}  // namespace modular_policy
}  // namespace stepit

#endif  // STEPIT_MODULAR_POLICY_MODULAR_POLICY_H_

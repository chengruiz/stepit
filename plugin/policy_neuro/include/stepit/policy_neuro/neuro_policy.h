#ifndef STEPIT_NEURO_POLICY_NEURO_POLICY_H_
#define STEPIT_NEURO_POLICY_NEURO_POLICY_H_

#include <set>

#include <stepit/policy.h>
#include <stepit/utils.h>
#include <stepit/policy_neuro/actuator.h>

namespace stepit {
namespace neuro_policy {
class NeuroPolicy : public Policy {
 public:
  NeuroPolicy(const RobotSpec &robot_spec, const std::string &home_dir);
  bool reset() override;
  bool act(const LowState &low_state, ControlRequests &requests, LowCmd &cmd) override;
  void exit() override;

 private:
  void addModule(Module::Ptr module, bool first);
  bool isSatisfied(const std::set<FieldId> &requirements) const;

  YAML::Node config_;
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
  ArrXf observation_, action_;
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_NEURO_POLICY_H_

#ifndef STEPIT_NEURO_POLICY_ACTION_SOURCE_H_
#define STEPIT_NEURO_POLICY_ACTION_SOURCE_H_

#include <string>

#include <stepit/utils.h>
#include <stepit/policy_neuro/field.h>

namespace stepit {
namespace neuro_policy {
class ActionHistory : public Module {
 public:
  ActionHistory(const PolicySpec &policy_spec, const std::string &home_dir);
  void initFieldProperties() override;
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &result) override;
  void postUpdate(const FieldMap &field_map) override;

 private:
  FieldId action_id_{};
  FieldId action_p1_id_{};
  FieldId action_p2_id_{};
  ArrXf action_mean_;
  StaticQueue<ArrXf> action_his_;
};

class ActionFilter : public Module {
 public:
  ActionFilter(const PolicySpec &policy_spec, const std::string &home_dir);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &result) override;

 private:
  int window_size_{};
  FieldId action_id_{};
  ArrXf action_mean_;
  StaticQueue<ArrXf> action_his_;
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_ACTION_SOURCE_H_

#ifndef STEPIT_NEURO_POLICY_ACTION_SOURCE_H_
#define STEPIT_NEURO_POLICY_ACTION_SOURCE_H_

#include <stepit/utils.h>
#include <stepit/policy_neuro/module.h>

namespace stepit {
namespace neuro_policy {
class ActionHistory : public Module {
 public:
  ActionHistory(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec);
  bool init() override;
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;
  void postStep(const FieldMap &context) override;

 private:
  FieldId action_id_{};
  FieldId last_action_id_{};
  FieldId action_p1_id_{};
  FieldId action_p2_id_{};
  ArrXf default_action_;
  RingBuffer<ArrXf> action_buf_;
};

class ActionFilter : public Module {
 public:
  ActionFilter(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec);
  bool init() override;
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;

 private:
  int window_size_{};
  FieldId action_id_{};
  ArrXf default_action_;

  RingBuffer<ArrXf> action_buf_;
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_ACTION_SOURCE_H_

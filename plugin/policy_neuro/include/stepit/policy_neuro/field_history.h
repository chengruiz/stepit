#ifndef STEPIT_NEURO_POLICY_FIELD_HISTORY_H_
#define STEPIT_NEURO_POLICY_FIELD_HISTORY_H_

#include <stepit/policy_neuro/field.h>

namespace stepit {
namespace neuro_policy {
class FieldHistoryBuffer {
 public:
  FieldHistoryBuffer() = default;
  explicit FieldHistoryBuffer(const std::pair<YAML::Node, YAML::Node> &node);

  void initFieldProperties();
  FieldId getSourceId() const { return source_id_; }
  FieldId getTargetId() const { return target_id_; }
  void clear() { history_.clear(); }
  const ArrXf &update(const ArrXf &frame);

 private:
  void push(const ArrXf &frame);
  void updateOutput();

  std::string source_name_;
  std::string target_name_;
  FieldId source_id_{};
  FieldId target_id_{};
  std::uint32_t history_len_{};
  FieldSize source_size_{};
  /* If true, the most recent entry will be placed at the beginning of the output vector.
   * Otherwise, it will be placed at the end. */
  bool newest_first_{true};
  bool include_current_frame_{true};
  ArrXf default_value_;

  RingBuffer<ArrXf> history_;
  ArrXf output_;
};

class FieldHistory : public Module {
 public:
  FieldHistory(const NeuroPolicySpec &policy_spec, const std::string &name);
  void initFieldProperties() override;
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;

 private:
  std::vector<FieldHistoryBuffer> buffers_;
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_FIELD_HISTORY_H_

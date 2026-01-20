#ifndef STEPIT_NEURO_POLICY_HISTORY_BUFFER_H_
#define STEPIT_NEURO_POLICY_HISTORY_BUFFER_H_

#include <deque>
#include <stepit/policy_neuro/field.h>

namespace stepit {
namespace neuro_policy {
class HistoryBuffer : public FieldSource {
 public:
  HistoryBuffer(const PolicySpec &policy_spec, const std::string &home_dir);
  void initFieldProperties() override;
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &result) override;

 private:
  YAML::Node config_;
  
  struct BufferConfig {
    FieldId source_id;
    FieldId target_id;
    std::uint32_t history_len;
    std::uint32_t source_size;
    bool newest_first;  // true: newest->oldest, false: oldest->newest
    std::deque<VecXf> history;
  };
  
  std::vector<BufferConfig> buffers_;
  VecXf output_buffer_;
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_HISTORY_BUFFER_H_

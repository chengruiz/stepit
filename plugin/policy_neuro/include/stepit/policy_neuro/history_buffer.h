#ifndef STEPIT_NEURO_POLICY_HISTORY_BUFFER_H_
#define STEPIT_NEURO_POLICY_HISTORY_BUFFER_H_

#include <stepit/policy_neuro/field.h>

namespace stepit {
namespace neuro_policy {
class HistoryBuffer : public Module {
 public:
  HistoryBuffer(const PolicySpec &policy_spec, const std::string &home_dir);
  void initFieldProperties() override;
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &result) override;

 private:
  struct BufferConfig {
    std::string source_name;
    std::string target_name;
    FieldId source_id{};
    FieldId target_id{};
    std::uint32_t history_len{};
    std::uint32_t source_size{};
    /* If true, the most recent entry will be placed at the beginning of the output vector.
     * Otherwise, it will be placed at the end. */
    bool newest_first{true};
    VecXf default_value;

    RingBuffer<VecXf> history;
    VecXf output_buffer;
  };

  YAML::Node config_;
  std::vector<BufferConfig> buffers_;
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_HISTORY_BUFFER_H_

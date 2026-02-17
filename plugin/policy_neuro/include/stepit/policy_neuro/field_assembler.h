#ifndef STEPIT_NEURO_POLICY_FIELD_ASSEMBLER_H_
#define STEPIT_NEURO_POLICY_FIELD_ASSEMBLER_H_

#include <stepit/utils.h>
#include <stepit/policy_neuro/field.h>

namespace stepit {
namespace neuro_policy {
class FieldAssembler : public Module {
 public:
  FieldAssembler(const PolicySpec &, const std::string &home_dir);
  void initFieldProperties() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &result) override;

 private:
  YAML::Node config_;
  FieldIdVec target_ids_;
  std::vector<FieldSize> target_sizes_;
  std::vector<FieldIdVec> component_ids_;

  std::vector<VecXf> buffers_;
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_FIELD_ASSEMBLER_H_

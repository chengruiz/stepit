#ifndef STEPIT_NEURO_POLICY_SCALING_SOURCE_H_
#define STEPIT_NEURO_POLICY_SCALING_SOURCE_H_

#include <utility>

#include <stepit/utils.h>
#include <stepit/policy_neuro/field.h>

namespace stepit {
namespace neuro_policy {
class FieldScaling : public Module {
 public:
  FieldScaling(const PolicySpec &policy_spec, const std::string &home_dir);
  void initFieldProperties() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &result) override;

 private:
  YAML::Node config_;
  std::vector<FieldId> field_ids_{};
  std::vector<std::pair<ArrXf, ArrXf>> scalings_{};
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_SCALING_SOURCE_H_

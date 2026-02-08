#ifndef STEPIT_NEURO_POLICY_HEIGHTMAP_SOURCE_H_
#define STEPIT_NEURO_POLICY_HEIGHTMAP_SOURCE_H_

#include <stepit/policy_neuro/field.h>

namespace stepit {
namespace neuro_policy {
class DummyHeightmapSource : public Module {
 public:
  DummyHeightmapSource(const PolicySpec &policy_spec, const std::string &home_dir);
  bool reset() override { return true; }
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &result) override;

 protected:
  YAML::Node config_;
  FieldId heightmap_id_;
  FieldId uncertainty_id_;

  std::size_t numHeightSamples() const { return sample_coords_.size(); }
  std::vector<Vec2f> sample_coords_;
  float default_uncertainty_{0.05F}, max_uncertainty_{0.5F};

  ArrXf elevation_, uncertainty_;
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_HEIGHTMAP_SOURCE_H_

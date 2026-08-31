#ifndef STEPIT_MODULAR_POLICY_LOCOMOTION_HEIGHTMAP_SOURCE_H_
#define STEPIT_MODULAR_POLICY_LOCOMOTION_HEIGHTMAP_SOURCE_H_

#include <stepit/modular_policy/module.h>

namespace stepit {
namespace modular_policy {
class DummyHeightmapSource : public Module {
 public:
  DummyHeightmapSource(const ModularPolicySpec &policy_spec, const ModuleSpec &module_spec);
  bool reset() override { return true; }
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;

 protected:
  FieldId heightmap_id_;
  FieldId uncertainty_id_;

  std::size_t numHeightSamples() const { return sample_coords_.size(); }
  std::vector<Vec2f> sample_coords_;
  float default_uncertainty_{}, max_uncertainty_{};

  ArrXf elevation_, uncertainty_;
};
}  // namespace modular_policy
}  // namespace stepit

#endif  // STEPIT_MODULAR_POLICY_LOCOMOTION_HEIGHTMAP_SOURCE_H_

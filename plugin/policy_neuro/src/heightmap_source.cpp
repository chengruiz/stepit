#include <stepit/policy_neuro/heightmap_source.h>

namespace stepit {
namespace neuro_policy {
DummyHeightmapSource::DummyHeightmapSource(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : Module(policy_spec, ModuleSpec(module_spec, "heightmap")) {
  if (config_["sample_coord"].hasValue()) {
    config_["sample_coord"].to(sample_coords_);
  } else {
    STEPIT_ASSERT(config_["dimension"].hasValue() and config_["grid_size"].hasValue(),
                  "Either 'sample_coord' or both 'dimension' and 'grid_size' must be specified in {}.",
                  config_filename_);
    std::array<int, 2> dimension;
    std::array<float, 2> grid_size;
    config_["dimension"].to(dimension);
    config_["grid_size"].to(grid_size);
    bool x_major = config_["x_major"].as<bool>(true);

    int dim_x = dimension[0], dim_y = dimension[1];
    float x0 = -(dim_x - 1) / 2.0F * grid_size[0];
    float y0 = -(dim_y - 1) / 2.0F * grid_size[1];

    sample_coords_.resize(dim_x * dim_y);
    for (int i{}; i < dim_x; ++i) {
      for (int j{}; j < dim_y; ++j) {
        sample_coords_[x_major ? i * dim_y + j : j * dim_x + i] = {x0 + i * grid_size[0], y0 + j * grid_size[1]};
      }
    }
  }

  default_uncertainty_ = config_["default_uncertainty"].as<float>(0.05F);
  max_uncertainty_     = config_["max_uncertainty"].as<float>(0.5F);

  elevation_.setZero(numHeightSamples());
  uncertainty_.setConstant(numHeightSamples(), max_uncertainty_);

  heightmap_id_   = registerProvision("heightmap", numHeightSamples());
  uncertainty_id_ = registerProvision("heightmap_uncertainty", numHeightSamples());
}

bool DummyHeightmapSource::update(const LowState &low_state, ControlRequests &requests, FieldMap &context) {
  context[heightmap_id_]   = elevation_;
  context[uncertainty_id_] = uncertainty_;
  return true;
}

STEPIT_REGISTER_MODULE(dummy_heightmap_source, kMinPriority, Module::make<DummyHeightmapSource>);
STEPIT_REGISTER_FIELD_SOURCE(heightmap, kMinPriority, Module::make<DummyHeightmapSource>);
STEPIT_REGISTER_FIELD_SOURCE(heightmap_uncertainty, kMinPriority, Module::make<DummyHeightmapSource>);
}  // namespace neuro_policy
}  // namespace stepit

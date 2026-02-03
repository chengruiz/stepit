#include <stepit/policy_neuro/heightmap_source.h>

namespace stepit {
namespace neuro_policy {
DummyHeightmapSource::DummyHeightmapSource(const PolicySpec &, const std::string &home_dir)
    : config_{YAML::LoadFile(home_dir + "/heightmap.yml")} {
  if (config_["dimension"] and config_["grid_size"]) {
    std::array<int, 2> dimension;
    std::array<float, 2> grid_size;
    bool x_major = true;
    yml::setTo(config_, "dimension", dimension);
    yml::setTo(config_, "grid_size", grid_size);
    yml::setIf(config_, "x_major", x_major);

    int dim_x = dimension[0], dim_y = dimension[1];
    float x0 = -(dim_x - 1) / 2.0F * grid_size[0];
    float y0 = -(dim_y - 1) / 2.0F * grid_size[1];

    sample_coords_.resize(dim_x * dim_y);
    for (int i{}; i < dim_x; ++i) {
      for (int j{}; j < dim_y; ++j) {
        sample_coords_[x_major ? i * dim_y + j : j * dim_x + i] = {x0 + i * grid_size[0], y0 + j * grid_size[1]};
      }
    }
  } else {
    yml::setTo(config_, "sample_coord", sample_coords_);
  }

  yml::setIf(config_, "default_uncertainty", default_uncertainty_);
  yml::setIf(config_, "max_uncertainty", max_uncertainty_);

  elevation_.setZero(numHeightSamples());
  uncertainty_.setConstant(numHeightSamples(), max_uncertainty_);

  heightmap_id_   = registerProvision("heightmap", numHeightSamples());
  uncertainty_id_ = registerProvision("heightmap_uncertainty", numHeightSamples());
}

bool DummyHeightmapSource::update(const LowState &low_state, ControlRequests &requests, FieldMap &result) {
  result[heightmap_id_]   = elevation_;
  result[uncertainty_id_] = uncertainty_;
  return true;
}

STEPIT_REGISTER_MODULE(heightmap, kMinPriority, Module::make<DummyHeightmapSource>);
STEPIT_REGISTER_FIELD_SOURCE(heightmap, kMinPriority, Module::make<DummyHeightmapSource>);
STEPIT_REGISTER_FIELD_SOURCE(heightmap_uncertainty, kMinPriority, Module::make<DummyHeightmapSource>);
}  // namespace neuro_policy
}  // namespace stepit

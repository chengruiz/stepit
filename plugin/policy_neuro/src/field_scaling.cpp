#include <stepit/policy_neuro/field_scaling.h>

namespace stepit {
namespace neuro_policy {
FieldScaling::FieldScaling(const PolicySpec &policy_spec, const std::string &home_dir)
    : config_(yml::loadFile(home_dir + "/field_scaling.yml")) {
  STEPIT_ASSERT(config_.IsMap(), "'field_scaling.yml' must contain a map of field scaling configurations.");
  for (const auto &node : config_) {
    field_ids_.push_back(registerRequirement(node.first.as<std::string>()));
  }
}

void FieldScaling::initFieldProperties() {
  for (const auto &node : config_) {
    auto field_id  = registerRequirement(node.first.as<std::string>());
    auto field_dim = getFieldSize(field_id);
    ArrXf scale{ArrXf::Ones(field_dim)}, bias{ArrXf::Zero(field_dim)};
    ArrXf mean{ArrXf::Ones(field_dim)}, std{ArrXf::Zero(field_dim)};

    if (yml::isValid(node.second, "bias")) {
      yml::setTo(node.second, "bias", bias);
    } else if (yml::isValid(node.second, "mean")) {
      yml::setTo(node.second, "mean", mean);
      bias = -mean;
    }

    if (yml::isValid(node.second, "scale")) {
      yml::setTo(node.second, "scale", scale);
    } else if (yml::isValid(node.second, "std")) {
      yml::setTo(node.second, "std", std);
      scale = std.cwiseInverse();
    }

    scalings_.emplace_back(scale, bias);
  }
}

bool FieldScaling::update(const LowState &, ControlRequests &, FieldMap &result) {
  for (std::size_t i{0}; i < field_ids_.size(); ++i) {
    auto &field         = result.at(field_ids_[i]);
    const auto &scaling = scalings_[i];
    field               = field.cwiseProduct(scaling.first) + scaling.second;
  }
  return true;
}

STEPIT_REGISTER_MODULE(field_scaling, kDefPriority, Module::make<FieldScaling>);
}  // namespace neuro_policy
}  // namespace stepit

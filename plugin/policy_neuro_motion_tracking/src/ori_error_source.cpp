#include <stepit/policy_neuro/ori_error_source.h>

namespace stepit {
namespace neuro_policy {
OriErrorSource::OriErrorSource(const NeuroPolicySpec &policy_spec, const std::string &name)
    : Module(nonEmptyOr(name, "ori_error")), config_(loadConfigIf(policy_spec)) {
  current_ori_name_ = yml::readIf<std::string>(config_, "current_ori_name", "base_global_ori");
  target_ori_name_  = yml::readIf<std::string>(config_, "target_ori_name", "base_target_ori");
  auto rot6d_order  = yml::readIf<std::string>(config_, "rotation_6d_order", "row_major");
  if (rot6d_order == "row_major") {
    rot6d_order_ = Rotation6dOrder::kRowMajor;
  } else if (rot6d_order == "column_major") {
    rot6d_order_ = Rotation6dOrder::kColumnMajor;
  } else {
    STEPIT_ERROR("Unsupported 'rotation_6d_order': '{}'. Expected 'column_major' or 'row_major'.", rot6d_order);
  }

  current_ori_id_  = registerRequirement(current_ori_name_, 4);
  target_ori_id_   = registerRequirement(target_ori_name_, 4);
  ori_error_id_    = registerProvision("ori_error", 4);
  ori_error_6d_id_ = registerProvision("ori_error_6d", 6);
}

bool OriErrorSource::update(const LowState &, ControlRequests &, FieldMap &context) {
  Quatf current_ori(context.at(current_ori_id_));
  Quatf target_ori(context.at(target_ori_id_));
  Quatf ori_error        = current_ori.inverse() * target_ori;
  context[ori_error_id_] = ori_error.coeffs();
  Mat3f rotation_matrix  = ori_error.matrix();
  Vec6f ori_error_6d;
  if (rot6d_order_ == Rotation6dOrder::kColumnMajor) {
    ori_error_6d << rotation_matrix.col(0), rotation_matrix.col(1);
  } else {
    // clang-format off
    ori_error_6d <<
        rotation_matrix(0, 0), rotation_matrix(0, 1),
        rotation_matrix(1, 0), rotation_matrix(1, 1),
        rotation_matrix(2, 0), rotation_matrix(2, 1);
    // clang-format on
  }
  context[ori_error_6d_id_] = ori_error_6d;
  return true;
}

STEPIT_REGISTER_MODULE(ori_error, kDefPriority, Module::make<OriErrorSource>);
STEPIT_REGISTER_FIELD_SOURCE(ori_error, kDefPriority, Module::make<OriErrorSource>);
STEPIT_REGISTER_FIELD_SOURCE(ori_error_6d, kDefPriority, Module::make<OriErrorSource>);
}  // namespace neuro_policy
}  // namespace stepit

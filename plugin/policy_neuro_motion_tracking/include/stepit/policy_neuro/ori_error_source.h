#ifndef STEPIT_NEURO_POLICY_ORI_ERROR_SOURCE_H_
#define STEPIT_NEURO_POLICY_ORI_ERROR_SOURCE_H_

#include <stepit/policy_neuro/field.h>

namespace stepit {
namespace neuro_policy {
class OriErrorSource : public Module {
 public:
  OriErrorSource(const NeuroPolicySpec &, const std::string &name);
  bool update(const LowState &, ControlRequests &, FieldMap &context) override;

  enum Rotation6dOrder { kRowMajor, kColumnMajor };

 protected:
  YAML::Node config_;
  std::string current_ori_name_;
  std::string target_ori_name_;
  FieldId current_ori_id_{};
  FieldId target_ori_id_{};
  FieldId ori_error_id_{};
  FieldId ori_error_6d_id_{};
  Rotation6dOrder rot6d_order_{kRowMajor};
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_ORI_ERROR_SOURCE_H_

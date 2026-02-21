#ifndef STEPIT_NEURO_POLICY_CONST_FIELD_SOURCE_H_
#define STEPIT_NEURO_POLICY_CONST_FIELD_SOURCE_H_

#include <string>
#include <vector>

#include <stepit/policy_neuro/field.h>

namespace stepit {
namespace neuro_policy {
class ConstFieldSource : public Module {
 public:
  ConstFieldSource(const NeuroPolicySpec &, const std::string &name);
  bool update(const LowState &, ControlRequests &, FieldMap &context) override;

 private:
  struct ConstField {
    std::string name;
    FieldId id{};
    FieldSize size{};
    ArrXf value;
  };

  YAML::Node config_;
  std::vector<ConstField> fields_;
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_CONST_FIELD_SOURCE_H_

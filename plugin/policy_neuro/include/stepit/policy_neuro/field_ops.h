#ifndef STEPIT_NEURO_POLICY_FIELD_OPS_H_
#define STEPIT_NEURO_POLICY_FIELD_OPS_H_

#include <stepit/policy_neuro/field.h>

namespace stepit {
namespace neuro_policy {
class FieldOps : public Module {
 public:
  FieldOps(const NeuroPolicySpec &policy_spec, const std::string &name);
  void initFieldProperties() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;

 private:
  enum class OpType {
    kAffine,
    kConcat,
    kSplit,
    kSlice,
    kCopy,
  };

  struct Operation {
    OpType type{};
    YAML::Node node;

    FieldId source_id{};
    FieldId target_id{};
    FieldIdVec source_ids;
    FieldIdVec target_ids;

    std::vector<FieldSize> segment_sizes;
    std::vector<FieldSize> indices;

    ArrXf scale;
    ArrXf bias;
    ArrXf buffer;
  };

  YAML::Node config_;
  std::vector<Operation> operations_;
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_FIELD_OPS_H_

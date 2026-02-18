#ifndef STEPIT_NEURO_POLICY_NEURO_MODULE_H_
#define STEPIT_NEURO_POLICY_NEURO_MODULE_H_

#include <stepit/nnrt/nnrt.h>
#include <stepit/policy_neuro/field.h>

namespace stepit {
namespace neuro_policy {
class NeuroModule : public Module {
 public:
  NeuroModule(const std::string &name, const std::string &home_dir);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;

 private:
  using FieldNameVec = std::vector<std::string>;
  using FieldSizeVec = std::vector<FieldSize>;
  static FieldId addField(const YAML::Node &node, FieldNameVec &field_names, FieldSizeVec &field_sizes);
  void parseFields(const std::string &key, const FieldNameVec &node_names, std::vector<FieldNameVec> &field_names,
                   std::vector<FieldSizeVec> &field_sizes, FieldSizeVec &total_dims, std::vector<FieldIdVec> &fields);

  NnrtApi::Ptr nn_;
  YAML::Node config_;

  std::string run_name_{};
  bool assert_all_finite_{true};
  FieldSizeVec input_dims_{}, output_dims_{};
  FieldNameVec input_names_{}, output_names_{};
  std::vector<FieldNameVec> input_field_names_, output_field_names_;
  std::vector<FieldSizeVec> input_field_sizes_, output_field_sizes_;
  std::vector<FieldIdVec> input_fields_, output_fields_;
  std::vector<ArrXf> input_arr_;
};

class NeuroActor : public NeuroModule {
 public:
  NeuroActor(const PolicySpec &policy_spec, const std::string &home_dir);
};

class NeuroEstimator : public NeuroModule {
 public:
  NeuroEstimator(const PolicySpec &policy_spec, const std::string &home_dir);
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_NEURO_MODULE_H_

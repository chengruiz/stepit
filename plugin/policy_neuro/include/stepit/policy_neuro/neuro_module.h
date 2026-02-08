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
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &result) override;

 private:
  static FieldId addField(const YAML::Node &node, std::vector<std::string> &field_names,
                          std::vector<std::uint32_t> &field_dims);
  void parseFields(const std::string &key, const std::vector<std::string> &node_names,
                   std::vector<std::vector<std::string>> &field_names,
                   std::vector<std::vector<std::uint32_t>> &field_dims, std::vector<std::uint32_t> &total_dims,
                   std::vector<std::vector<FieldId>> &fields);

  NnrtApi::Ptr nn_;
  YAML::Node config_;

  std::string run_name_{};
  bool assert_all_finite_{true};
  std::vector<std::uint32_t> input_dims_{}, output_dims_{};
  std::vector<std::string> input_names_{}, output_names_{};
  std::vector<std::vector<std::string>> input_field_names_, output_field_names_;
  std::vector<std::vector<std::uint32_t>> input_field_dims_, output_field_dims_;
  std::vector<std::vector<FieldId>> input_fields_, output_fields_;
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

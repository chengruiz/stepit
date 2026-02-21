#ifndef STEPIT_NEURO_POLICY_NEURO_MODULE_H_
#define STEPIT_NEURO_POLICY_NEURO_MODULE_H_

#include <stepit/nnrt/nnrt.h>
#include <stepit/policy_neuro/field.h>

namespace stepit {
namespace neuro_policy {
class NeuroModule : public Module {
 public:
  NeuroModule(const NeuroPolicySpec &policy_spec, const std::string &name);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;

 private:
  using FieldNameVec = std::vector<std::string>;
  using FieldSizeVec = std::vector<FieldSize>;
  void parseFields(                            // Parse field properties from the configuration
      bool is_input,                           // true for input fields, false for output fields
      const FieldNameVec &node_names,          // Names of the neural network's ordinary inputs/outputs
      const FieldSizeVec &node_sizes,          // Sizes of the neural network's ordinary inputs/outputs
      std::vector<FieldNameVec> &field_names,  // Output vector of field names for each ordinary input/output
      std::vector<FieldSizeVec> &field_sizes,  // Output vector of field sizes for each ordinary input/output
      std::vector<FieldIdVec> &fields_ids      // Output vector of field IDs for each ordinary input/output
  );
  static void printNodeFields(const std::vector<std::string> &node_names, const std::vector<FieldIdVec> &field_ids);

  NnrtApi::Ptr nn_;
  YAML::Node config_;

  std::string run_name_{};
  bool assert_all_finite_{true};
  FieldSizeVec input_dims_{}, output_dims_{};
  FieldNameVec input_names_{}, output_names_{};
  std::vector<FieldNameVec> input_field_names_, output_field_names_;
  std::vector<FieldSizeVec> input_field_sizes_, output_field_sizes_;
  std::vector<FieldIdVec> input_field_ids_, output_field_ids_;
  std::vector<ArrXf> input_arr_;
};

class NeuroActor : public NeuroModule {
 public:
  NeuroActor(const NeuroPolicySpec &policy_spec, const std::string &name);
};

class NeuroEstimator : public NeuroModule {
 public:
  NeuroEstimator(const NeuroPolicySpec &policy_spec, const std::string &name);
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_NEURO_MODULE_H_

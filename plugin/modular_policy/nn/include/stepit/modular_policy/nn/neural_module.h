#ifndef STEPIT_MODULAR_POLICY_NN_NEURAL_MODULE_H_
#define STEPIT_MODULAR_POLICY_NN_NEURAL_MODULE_H_

#include <stepit/nnrt/nnrt.h>
#include <stepit/modular_policy/module.h>

namespace stepit {
namespace modular_policy {
class NeuralModule : public Module {
 public:
  NeuralModule(const ModularPolicySpec &policy_spec, const ModuleSpec &module_spec);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;

 private:
  using FieldNameVec = std::vector<std::string>;
  using DataTypeVec  = std::vector<DataType>;
  using FieldSizeVec = std::vector<FieldSize>;
  void parseFields(                            // Parse field properties from the configuration
      bool is_input,                           // true for input fields, false for output fields
      const FieldNameVec &node_names,          // Names of the neural network's ordinary inputs/outputs
      const DataTypeVec &node_dtypes,          // Scalar types of the neural network's ordinary inputs/outputs
      const FieldSizeVec &node_sizes,          // Sizes of the neural network's ordinary inputs/outputs
      std::vector<FieldNameVec> &field_names,  // Output vector of field names for each ordinary input/output
      std::vector<FieldSizeVec> &field_sizes,  // Output vector of field sizes for each ordinary input/output
      std::vector<FieldIdVec> &field_ids       // Output vector of field IDs for each ordinary input/output
  );
  static void printNodeFields(const std::vector<std::string> &node_names, const std::vector<FieldIdVec> &field_ids);

  Nnrt::Ptr nn_;

  std::string nnrt_factory_;
  std::string model_path_;
  std::string run_name_;
  bool assert_all_finite_{true};
  DataTypeVec input_dtypes_, output_dtypes_;
  FieldSizeVec input_dims_{}, output_dims_{};
  FieldNameVec input_names_{}, output_names_{};
  std::vector<FieldNameVec> input_field_names_, output_field_names_;
  std::vector<FieldSizeVec> input_field_sizes_, output_field_sizes_;
  std::vector<FieldIdVec> input_field_ids_, output_field_ids_;
  std::vector<FieldValue> input_values_;
};

class NeuralActor : public NeuralModule {
 public:
  NeuralActor(const ModularPolicySpec &policy_spec, const ModuleSpec &module_spec);
};

class NeuralEstimator : public NeuralModule {
 public:
  NeuralEstimator(const ModularPolicySpec &policy_spec, const ModuleSpec &module_spec);
};
}  // namespace modular_policy
}  // namespace stepit

#endif  // STEPIT_MODULAR_POLICY_NN_NEURAL_MODULE_H_

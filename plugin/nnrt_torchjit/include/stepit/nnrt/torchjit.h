#ifndef STEPIT_NNRT_TORCHJIT_H_
#define STEPIT_NNRT_TORCHJIT_H_

#include <string>
#include <vector>

#include <torch/script.h>

#include <stepit/nnrt/nnrt.h>

namespace stepit {
class TorchJitApi : public NnrtApi {
 public:
  explicit TorchJitApi(const std::string &path, const YAML::Node &config);
  ~TorchJitApi() override = default;

  void runInference() override;
  void clearState() override;

  using NnrtApi::setInput;
  void setInput(std::size_t idx, float *data) override;
  using NnrtApi::getOutput;
  const float *getOutput(std::size_t idx) override;

 private:
  static std::vector<torch::Tensor> extractOutputTensors(const torch::jit::IValue &output);
  static torch::Tensor normalizeTensor(const torch::Tensor &tensor);
  void initInputSpec();
  void initOutputSpec();

  torch::jit::script::Module module_;
  std::vector<std::vector<float>> in_data_, out_data_;
};
}  // namespace stepit

#endif  // STEPIT_NNRT_TORCHJIT_H_

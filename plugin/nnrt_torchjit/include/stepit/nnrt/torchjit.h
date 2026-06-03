#ifndef STEPIT_NNRT_TORCHJIT_H_
#define STEPIT_NNRT_TORCHJIT_H_

#include <cstdint>
#include <string>
#include <vector>

#include <torch/script.h>

#include <stepit/nnrt/nnrt.h>

namespace stepit {
class TorchJit : public Nnrt {
 public:
  explicit TorchJit(const std::string &path, const yml::Node &config);
  ~TorchJit() override = default;

  void runInference() override;
  void clearState() override;

  using Nnrt::setInput;
  void setInput(std::size_t idx, const void *data) override;
  using Nnrt::getOutput;
  const void *getOutput(std::size_t idx) override;

 private:
  static std::vector<torch::Tensor> extractOutputTensors(const torch::jit::IValue &output);
  static torch::Tensor normalizeTensor(const torch::Tensor &tensor);
  static DataType mapTorchDtype(torch::ScalarType scalar_type);
  static torch::ScalarType toTorchDtype(DataType dtype);
  void initInputSpec();
  void initOutputSpec();

  torch::jit::script::Module module_;
  std::vector<std::vector<uint8_t>> in_data_, out_data_;
};
}  // namespace stepit

#endif  // STEPIT_NNRT_TORCHJIT_H_

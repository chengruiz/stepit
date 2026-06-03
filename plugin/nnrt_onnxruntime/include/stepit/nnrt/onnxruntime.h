#ifndef STEPIT_NNRT_ONNXRUNTIME_H_
#define STEPIT_NNRT_ONNXRUNTIME_H_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

#include <stepit/nnrt/nnrt.h>

namespace stepit {
class OnnxRt : public Nnrt {
 public:
  explicit OnnxRt(const std::string &path, const yml::Node &config);
  ~OnnxRt() override = default;

  void runInference() override;
  void clearState() override;

  using Nnrt::setInput;
  void setInput(std::size_t idx, const void *data) override;
  using Nnrt::getOutput;
  const void *getOutput(std::size_t idx) override;

 private:
  static DataType mapOnnxDtype(ONNXTensorElementDataType onnx_type);
  Ort::Value createTensor(void *data, std::size_t byte_size, const std::vector<int64_t> &shape, DataType dtype);

  Ort::Env env_;
  Ort::AllocatorWithDefaultOptions allocator_;
  Ort::MemoryInfo memory_info_{nullptr};
  Ort::RunOptions run_options_{nullptr};
  std::unique_ptr<Ort::Session> core_{nullptr};
  std::vector<std::vector<uint8_t>> in_data_, out_data_;
  std::vector<Ort::Value> in_tensors_, out_tensors_;
  std::unique_ptr<Ort::IoBinding> io_binding_{nullptr};
};
}  // namespace stepit

#endif  // STEPIT_NNRT_ONNXRUNTIME_H_

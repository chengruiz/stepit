#ifndef STEPIT_NNRT_ONNXRUNTIME_H_
#define STEPIT_NNRT_ONNXRUNTIME_H_

#include <memory>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

#include <stepit/nnrt/nnrt.h>

namespace stepit {
class OnnxrtApi : public NnrtApi {
 public:
  explicit OnnxrtApi(const std::string &path, const YAML::Node &config);
  ~OnnxrtApi() override = default;

  void runInference() override;
  void clearState() override;

  using NnrtApi::setInput;
  void setInput(std::size_t idx, float *data) override;
  using NnrtApi::getOutput;
  const float *getOutput(std::size_t idx) override;

 private:
  template <typename T>
  Ort::Value createTensor(T *data, size_t size, const std::vector<int64_t> &shape) const;
  static std::vector<int64_t> getShapeFromTypeInfo(const Ort::TypeInfo &type_info);

  Ort::Env env_;
  Ort::AllocatorWithDefaultOptions allocator_;
  Ort::MemoryInfo memory_info_{nullptr};
  Ort::RunOptions run_options_{nullptr};
  std::unique_ptr<Ort::Session> core_{nullptr};
  std::vector<std::vector<float>> in_data_, out_data_;
  std::vector<Ort::Value> in_tensors_, out_tensors_;
  std::unique_ptr<Ort::IoBinding> io_binding_{nullptr};
};

template <typename T>
Ort::Value OnnxrtApi::createTensor(T *data, size_t size, const std::vector<int64_t> &shape) const {
  return Ort::Value::CreateTensor<T>(memory_info_, data, size, shape.data(), shape.size());
}
}  // namespace stepit

#endif  // STEPIT_NNRT_ONNXRUNTIME_H_

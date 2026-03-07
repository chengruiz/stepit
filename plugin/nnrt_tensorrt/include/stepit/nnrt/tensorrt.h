#ifndef STEPIT_NNRT_TENSORRT_H_
#define STEPIT_NNRT_TENSORRT_H_

#include <memory>
#include <string>
#include <vector>

#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime.h>

#include <stepit/logging.h>
#include <stepit/nnrt/nnrt.h>

namespace stepit {
class CuMemory {
 public:
  explicit CuMemory(std::size_t size);
  explicit CuMemory(cudaStream_t stream, std::size_t size);
  CuMemory(const CuMemory &)            = delete;
  CuMemory &operator=(const CuMemory &) = delete;
  CuMemory(CuMemory &&other) noexcept;
  ~CuMemory();

  void *ptr() const { return ptr_; }
  operator void *() const { return ptr_; }
  std::size_t size() const { return size_; }

 private:
  void *ptr_{nullptr};
  cudaStream_t stream_{nullptr};
  std::size_t size_{};
};

class CuLogger : public nvinfer1::ILogger {
 public:
  static CuLogger &instance();

  void log(Severity severity, const char *msg) noexcept override {
    switch (severity) {
      case Severity::kINTERNAL_ERROR:
      case Severity::kERROR:
        STEPIT_CRITNT(msg);
        return;
      case Severity::kWARNING:
        STEPIT_WARNNT(msg);
        break;
      case Severity::kINFO:
        STEPIT_LOGNT(msg);
        break;
      default:
        STEPIT_DBUGNT(msg);
        break;
    }
  }
};

class TensorRTApi : public NnrtApi {
 public:
  explicit TensorRTApi(const std::string &path, const YAML::Node &config);
  ~TensorRTApi() override;

  void runInference() override;
  void clearState() override;

  using NnrtApi::setInput;
  void setInput(std::size_t idx, float *data) override;
  using NnrtApi::getOutput;
  const float *getOutput(std::size_t idx) override;

 private:
  void synchronize() override;
  bool build(const std::string &onnx_path, const std::string &engine_path);
  bool createCudaGraph();

  int device_id_{};
  bool use_fp16_{true};
  bool force_rebuild_{};
  std::string engine_path_;

  std::vector<CuMemory> inputs_, outputs_;
  std::vector<float *> out_data_;

  std::unique_ptr<nvinfer1::IRuntime> runtime_{nullptr};
  std::unique_ptr<nvinfer1::ICudaEngine> engine_{nullptr};
  std::unique_ptr<nvinfer1::IExecutionContext> context_{nullptr};
  cudaGraph_t cu_graph_{nullptr};
  cudaGraphExec_t cu_instance_{nullptr};
  cudaStream_t cu_stream_{nullptr};
};
}  // namespace stepit

#endif  // STEPIT_NNRT_TENSORRT_H_

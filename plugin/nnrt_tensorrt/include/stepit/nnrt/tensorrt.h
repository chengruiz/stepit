#ifndef STEPIT_NNRT_TENSORRT_H_
#define STEPIT_NNRT_TENSORRT_H_

#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime.h>

#include <stepit/logging.h>
#include <stepit/nnrt/nnrt.h>

namespace stepit {
struct CudaDeviceMemoryDeleter {
  void operator()(void *ptr) const;
};

struct CudaGraphDeleter {
  void operator()(cudaGraph_t graph) const;
};

struct CudaGraphExecDeleter {
  void operator()(cudaGraphExec_t instance) const;
};

struct CudaHostMemoryDeleter {
  void operator()(float *memory) const;
};

struct CudaStreamDeleter {
  void operator()(cudaStream_t stream) const;
};

using CudaDeviceMemoryPtr = std::unique_ptr<void, CudaDeviceMemoryDeleter>;
using CudaGraphPtr        = std::unique_ptr<std::remove_pointer_t<cudaGraph_t>, CudaGraphDeleter>;
using CudaGraphExecPtr    = std::unique_ptr<std::remove_pointer_t<cudaGraphExec_t>, CudaGraphExecDeleter>;
using CudaHostMemoryPtr   = std::unique_ptr<float, CudaHostMemoryDeleter>;
using CudaStreamPtr       = std::unique_ptr<std::remove_pointer_t<cudaStream_t>, CudaStreamDeleter>;

class TensorRTLogger : public nvinfer1::ILogger {
 public:
  static TensorRTLogger &instance();
  void log(Severity severity, const char *msg) noexcept override;
};

class TensorRTApi : public NnrtApi {
 public:
  explicit TensorRTApi(const std::string &path, const yml::Node &config);
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

  std::vector<CudaDeviceMemoryPtr> inputs_, outputs_;
  std::vector<CudaHostMemoryPtr> out_data_;

  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;
  CudaGraphPtr cuda_graph_;
  CudaGraphExecPtr cuda_instance_;
  CudaStreamPtr cuda_stream_;
};
}  // namespace stepit

#endif  // STEPIT_NNRT_TENSORRT_H_

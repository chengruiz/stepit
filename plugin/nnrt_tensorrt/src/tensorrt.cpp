#include <algorithm>
#include <fstream>
#include <iostream>
#include <numeric>

#include <stepit/nnrt/tensorrt.h>

#define STEPIT_CUDA_CALL(api, ...)                                            \
  do {                                                                        \
    auto ret = api(__VA_ARGS__);                                              \
    STEPIT_ASSERT(ret == cudaSuccess, #api " failed (error code: {}).", ret); \
  } while (0)

namespace stepit {
class CuStream {
 public:
  explicit CuStream() { STEPIT_CUDA_CALL(cudaStreamCreate, &stream_); }
  ~CuStream() { cudaStreamDestroy(stream_); }
  operator cudaStream_t() const { return stream_; }

 private:
  cudaStream_t stream_{nullptr};
};

CuMemory::CuMemory(std::size_t size) : size_(size) { STEPIT_CUDA_CALL(cudaMalloc, &ptr_, size); }

CuMemory::CuMemory(cudaStream_t stream, std::size_t size) : stream_(stream), size_(size) {
  STEPIT_CUDA_CALL(cudaMallocAsync, &ptr_, size, stream_);
}

CuMemory::CuMemory(CuMemory &&other) noexcept {
  ptr_       = other.ptr_;
  size_      = other.size_;
  other.ptr_ = nullptr;
}

CuMemory::~CuMemory() {
  if (stream_) {
    cudaFreeAsync(ptr_, stream_);
  } else {
    cudaFree(ptr_);
  }
}

CuLogger &CuLogger::instance() {
  static CuLogger instance;
  return instance;
}

TensorRTApi::TensorRTApi(const std::string &path, const YAML::Node &config)
    : NnrtApi(addExtensionIfMissing(path, ".onnx"), config) {
  STEPIT_CUDA_CALL(cudaStreamCreate, &cu_stream_);
  std::string engine_path = replaceExtension(path_, ".engine");
  if (not std::ifstream(engine_path).good()) {
    STEPIT_ASSERT(build(path, engine_path), "Failed to build TensorRT engine!");
    STEPIT_LOGNT("Write engine to {}.", engine_path);
  }

  std::ifstream file(engine_path, std::ios::binary | std::ios::ate);
  std::streamsize filesize = file.tellg();
  file.seekg(0, std::ios::beg);

  std::vector<char> buffer(filesize);
  STEPIT_ASSERT(file.read(buffer.data(), filesize), "Failed to read engine file");

  runtime_ = std::unique_ptr<nvinfer1::IRuntime>{nvinfer1::createInferRuntime(CuLogger::instance())};
  STEPIT_ASSERT(runtime_, "Failed to create TensorRT runtime!");

  engine_ = std::unique_ptr<nvinfer1::ICudaEngine>(runtime_->deserializeCudaEngine(buffer.data(), buffer.size()));
  STEPIT_ASSERT(engine_, "Failed to deserialize TensorRT engine!");

  context_ = std::unique_ptr<nvinfer1::IExecutionContext>(engine_->createExecutionContext());
  STEPIT_ASSERT(context_, "Failed to create TensorRT execution context!");

  for (int i{}; i < engine_->getNbIOTensors(); ++i) {
    const char *name = engine_->getIOTensorName(i);
    auto dims        = engine_->getTensorShape(name);
    auto size        = std::accumulate(dims.d, dims.d + dims.nbDims, 1, std::multiplies<>());

    if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
      in_shapes_.emplace_back(dims.d, dims.d + dims.nbDims);
      in_sizes_.push_back(size);
      in_names_.push_back(name);
      inputs_.emplace_back(cu_stream_, size * sizeof(float));
      ++num_in_;
      STEPIT_ASSERT(context_->setTensorAddress(name, inputs_.back()), "Failed to set tensor address!");
    } else {
      out_shapes_.emplace_back(dims.d, dims.d + dims.nbDims);
      out_sizes_.push_back(size);
      float *memory{nullptr};
      cudaMallocHost(&memory, size * sizeof(float));
      out_data_.push_back(memory);
      out_names_.push_back(name);
      outputs_.emplace_back(cu_stream_, size * sizeof(float));
      ++num_out_;
      STEPIT_ASSERT(context_->setTensorAddress(name, outputs_.back()), "Failed to set tensor address!");
    }
  }
  createCudaGraph();

  postInit();
}

TensorRTApi::~TensorRTApi() {
  inputs_.clear();
  outputs_.clear();
  for (auto *ptr : out_data_) cudaFreeHost(ptr);
  cudaGraphExecDestroy(cu_instance_);
  cudaGraphDestroy(cu_graph_);
  runtime_.reset();
  context_.reset();
  engine_.reset();
  cudaStreamDestroy(cu_stream_);
}

bool TensorRTApi::build(const std::string &onnx_path, const std::string &engine_path) {
  auto builder = std::unique_ptr<nvinfer1::IBuilder>(nvinfer1::createInferBuilder(CuLogger::instance()));
  if (builder == nullptr) return false;

  const auto explicit_batch_flag = 1U << static_cast<uint32_t>(
                                       nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
  auto network = std::unique_ptr<nvinfer1::INetworkDefinition>(builder->createNetworkV2(explicit_batch_flag));
  if (network == nullptr) return false;

  auto parser = std::unique_ptr<nvonnxparser::IParser>(nvonnxparser::createParser(*network, CuLogger::instance()));
  if (parser == nullptr) return false;

  auto constructed = parser->parseFromFile(onnx_path.c_str(), 0);
  if (not constructed) return false;

  auto config = std::unique_ptr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());
  if (config == nullptr) return false;
  config->setFlag(nvinfer1::BuilderFlag::kFP16);

  auto plan = std::unique_ptr<nvinfer1::IHostMemory>{builder->buildSerializedNetwork(*network, *config)};
  if (plan == nullptr) return false;

  std::ofstream outfile(engine_path, std::ofstream::binary);
  outfile.write(reinterpret_cast<const char *>(plan->data()), plan->size());
  return true;
}

bool TensorRTApi::createCudaGraph() {
  CuStream graph_stream;

  if (cudaStreamBeginCapture(graph_stream, cudaStreamCaptureModeGlobal) != cudaSuccess) {
    STEPIT_WARNNT("During creating cuda graph: Failed to capture the cuda stream!");
    return false;
  }
  if (not context_->enqueueV3(graph_stream)) {
    STEPIT_WARNNT("During creating cuda graph: Failed to run inference!");
    return false;
  }
  if (cudaStreamEndCapture(graph_stream, &cu_graph_) != cudaSuccess) {
    STEPIT_WARNNT("During creating cuda graph: Failed to capture the cuda stream!");
    return false;
  }
  if (cudaGraphInstantiate(&cu_instance_, cu_graph_, nullptr, nullptr, 0) != cudaSuccess) {
    STEPIT_WARNNT("During creating cuda graph: Failed to instantiate the cuda graph!");
    return false;
  }
  return true;
}

void TensorRTApi::runInference() {
  if (cu_instance_) {
    STEPIT_CUDA_CALL(cudaGraphLaunch, cu_instance_, cu_stream_);
  } else {
    STEPIT_ASSERT(context_->enqueueV3(cu_stream_), "Failed to run inference!");
  }
  for (const auto &pair : recur_param_indices_) {
    STEPIT_CUDA_CALL(cudaMemcpyAsync, inputs_[pair.first], outputs_[pair.second], inputs_[pair.first].size(),
                     cudaMemcpyDeviceToDevice, cu_stream_);
  }
}

void TensorRTApi::clearState() {
  for (const auto &pair : recur_param_indices_) {
    STEPIT_CUDA_CALL(cudaMemsetAsync, inputs_[pair.first], 0, inputs_[pair.first].size(), cu_stream_);
  }
}

void TensorRTApi::synchronize() { cudaStreamSynchronize(cu_stream_); }

void TensorRTApi::setInput(std::size_t idx, float *data) {
  STEPIT_CUDA_CALL(cudaMemcpyAsync, inputs_[idx], data, inputs_[idx].size(), cudaMemcpyHostToDevice, cu_stream_);
}

const float *TensorRTApi::getOutput(std::size_t idx) {
  STEPIT_CUDA_CALL(cudaMemcpyAsync, out_data_[idx], outputs_[idx], outputs_[idx].size(), cudaMemcpyDeviceToHost,
                   cu_stream_);
  synchronize();
  return out_data_[idx];
}

STEPIT_REGISTER_NNRTAPI(tensorrt, kDefPriority, NnrtApi::make<TensorRTApi>);
}  // namespace stepit

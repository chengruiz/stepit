#include <cctype>
#include <fstream>
#include <iostream>
#include <numeric>
#include <type_traits>

#include <NvInferVersion.h>

#include <stepit/nnrt/tensorrt.h>

#define STEPIT_CUDA_CALL(api, ...)                                                        \
  do {                                                                                    \
    auto status = api(__VA_ARGS__);                                                       \
    STEPIT_ASSERT(status == cudaSuccess, #api " failed (error code: {}).",                \
                  static_cast<typename std::underlying_type<cudaError_t>::type>(status)); \
  } while (0)

#define TENSORRT_VERSION_AT_LEAST(major, minor, patch) \
  ((NV_TENSORRT_MAJOR > (major)) or                    \
   (NV_TENSORRT_MAJOR == (major) and                   \
    (NV_TENSORRT_MINOR > (minor) or (NV_TENSORRT_MINOR == (minor) and NV_TENSORRT_PATCH >= (patch)))))

#if !TENSORRT_VERSION_AT_LEAST(8, 5, 1)
#error "stepit_plugin_nnrt_tensorrt requires TensorRT 8.5.1 or newer."
#endif

namespace stepit {
namespace {
std::size_t tensorBytes(std::int64_t elements) { return static_cast<std::size_t>(elements) * sizeof(float); }

std::string dimsToString(const nvinfer1::Dims &dims) {
  using Dim = std::remove_reference_t<decltype(nvinfer1::Dims::d[0])>;
  std::vector<Dim> values(dims.d, dims.d + dims.nbDims);
  return fmt::format("{}", values);
}

bool hasDynamicDim(const nvinfer1::Dims &dims) {
  for (std::int32_t i{}; i < dims.nbDims; ++i) {
    if (dims.d[i] < 0) return true;
  }
  return false;
}

class CudaDeviceGuard {
 public:
  explicit CudaDeviceGuard(int device_id) : device_id_(device_id) {
    STEPIT_CUDA_CALL(cudaGetDevice, &previous_device_);
    if (previous_device_ != device_id_) {
      STEPIT_CUDA_CALL(cudaSetDevice, device_id_);
      restore_previous_device_ = true;
    }
  }

  ~CudaDeviceGuard() {
    if (restore_previous_device_) cudaSetDevice(previous_device_);
  }

 private:
  int device_id_{};
  int previous_device_{};
  bool restore_previous_device_{false};
};

CudaStreamPtr makeCudaStream() {
  cudaStream_t stream{nullptr};
  STEPIT_CUDA_CALL(cudaStreamCreate, &stream);
  return CudaStreamPtr(stream);
}

CudaDeviceMemoryPtr makeCudaDeviceMemory(std::size_t size) {
  void *memory{nullptr};
  STEPIT_CUDA_CALL(cudaMalloc, &memory, size);
  return CudaDeviceMemoryPtr(memory);
}

CudaHostMemoryPtr makeCudaHostMemory(std::size_t size) {
  float *memory{nullptr};
  STEPIT_CUDA_CALL(cudaMallocHost, &memory, size);
  return CudaHostMemoryPtr(memory);
}
}  // namespace

void CudaDeviceMemoryDeleter::operator()(void *ptr) const {
  if (ptr != nullptr) cudaFree(ptr);
}

void CudaGraphDeleter::operator()(cudaGraph_t graph) const {
  if (graph != nullptr) cudaGraphDestroy(graph);
}

void CudaGraphExecDeleter::operator()(cudaGraphExec_t instance) const {
  if (instance != nullptr) cudaGraphExecDestroy(instance);
}

void CudaHostMemoryDeleter::operator()(float *memory) const {
  if (memory != nullptr) cudaFreeHost(memory);
}

void CudaStreamDeleter::operator()(cudaStream_t stream) const {
  if (stream == nullptr) return;
  cudaStreamSynchronize(stream);
  cudaStreamDestroy(stream);
}

TensorRTLogger &TensorRTLogger::instance() {
  static TensorRTLogger instance;
  return instance;
}

void TensorRTLogger::log(Severity severity, const char *msg) noexcept {
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

TensorRTApi::TensorRTApi(const std::string &path, const yml::Node &config)
    : NnrtApi(addExtensionIfMissing(path, ".onnx"), config) {
  auto tensorrt_options = config["tensorrt_options"];

  device_id_     = tensorrt_options["device_id"].as<int>(0);
  force_rebuild_ = tensorrt_options["force_rebuild"].as<bool>(false);
  auto precision = toLowercase(tensorrt_options["precision"].as<std::string>("fp32"));
  if (precision == "fp16") {
    use_fp16_ = true;
  } else if (precision == "fp32") {
    use_fp16_ = false;
  } else {
    STEPIT_THROW("Unsupported TensorRT precision '{}'. Expected 'fp16' or 'fp32'.", precision);
  }

  engine_path_ = tensorrt_options["engine_path"].as<std::string>(replaceExtension(path_, ".engine"));
  if (not engine_path_.empty() and engine_path_[0] != '/') {
    engine_path_ = joinPaths(fs::path(path_).parent_path().string(), engine_path_);
  }

  CudaDeviceGuard device_guard(device_id_);
  cuda_stream_ = makeCudaStream();

  if (force_rebuild_ or not std::ifstream(engine_path_).good()) {
    STEPIT_ASSERT(build(path_, engine_path_), "Failed to build TensorRT engine!");
    STEPIT_LOGNT("Write engine to {}.", engine_path_);
  }

  std::ifstream file(engine_path_, std::ios::binary | std::ios::ate);
  STEPIT_ASSERT(file.is_open(), "Failed to open TensorRT engine file '{}'.", engine_path_);
  std::streamsize filesize = file.tellg();
  STEPIT_ASSERT(filesize > 0, "TensorRT engine file '{}' is empty or invalid.", engine_path_);
  file.seekg(0, std::ios::beg);
  STEPIT_ASSERT(file.good(), "Failed to seek TensorRT engine file '{}'.", engine_path_);

  std::vector<char> buffer(static_cast<std::size_t>(filesize));
  STEPIT_ASSERT(file.read(buffer.data(), filesize), "Failed to read TensorRT engine file '{}'.", engine_path_);

  runtime_ = std::unique_ptr<nvinfer1::IRuntime>{nvinfer1::createInferRuntime(TensorRTLogger::instance())};
  STEPIT_ASSERT(runtime_, "Failed to create TensorRT runtime!");

  engine_ = std::unique_ptr<nvinfer1::ICudaEngine>(runtime_->deserializeCudaEngine(buffer.data(), buffer.size()));
  STEPIT_ASSERT(engine_, "Failed to deserialize TensorRT engine!");

  context_ = std::unique_ptr<nvinfer1::IExecutionContext>(engine_->createExecutionContext());
  STEPIT_ASSERT(context_, "Failed to create TensorRT execution context!");

  for (int i{}; i < engine_->getNbIOTensors(); ++i) {
    const char *name = engine_->getIOTensorName(i);
    auto dims        = engine_->getTensorShape(name);
    STEPIT_ASSERT(not hasDynamicDim(dims),
                  "TensorRT tensor '{}' has dynamic shape {}. Use 'trtexec' for advanced dynamic-shape builds.", name,
                  dimsToString(dims));
    auto size  = std::accumulate(dims.d, dims.d + dims.nbDims, 1, std::multiplies<>());
    auto bytes = tensorBytes(size);

    if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
      in_shapes_.emplace_back(dims.d, dims.d + dims.nbDims);
      in_sizes_.push_back(size);
      in_names_.push_back(name);
      inputs_.emplace_back(makeCudaDeviceMemory(bytes));
      ++num_in_;
      STEPIT_ASSERT(context_->setTensorAddress(name, inputs_.back().get()), "Failed to set tensor address!");
    } else {
      out_shapes_.emplace_back(dims.d, dims.d + dims.nbDims);
      out_sizes_.push_back(size);
      out_data_.emplace_back(makeCudaHostMemory(bytes));
      out_names_.push_back(name);
      outputs_.emplace_back(makeCudaDeviceMemory(bytes));
      ++num_out_;
      STEPIT_ASSERT(context_->setTensorAddress(name, outputs_.back().get()), "Failed to set tensor address!");
    }
  }
  createCudaGraph();

  postInit();
}

TensorRTApi::~TensorRTApi() {
  CudaDeviceGuard device_guard(device_id_);
  if (cuda_stream_) STEPIT_CUDA_CALL(cudaStreamSynchronize, cuda_stream_.get());
  cuda_instance_.reset();
  cuda_graph_.reset();
  context_.reset();
  inputs_.clear();
  outputs_.clear();
  out_data_.clear();
  engine_.reset();
  runtime_.reset();
  cuda_stream_.reset();
}

bool TensorRTApi::build(const std::string &onnx_path, const std::string &engine_path) {
  auto builder = std::unique_ptr<nvinfer1::IBuilder>(nvinfer1::createInferBuilder(TensorRTLogger::instance()));
  if (builder == nullptr) return false;

  nvinfer1::NetworkDefinitionCreationFlags flags{0U};
#if !TENSORRT_VERSION_AT_LEAST(10, 0, 0)
  flags |= 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
#endif
#if TENSORRT_VERSION_AT_LEAST(10, 12, 0)
  if (use_fp16_) {
    STEPIT_WARNNT(
        "TensorRT 10.12+ builds the ONNX model with strongly typed mode for 'precision: fp16'. Use an FP16 ONNX model "
        "or a prebuilt FP16 engine if you require FP16 inference.");
    flags |= 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kSTRONGLY_TYPED);
  }
#endif

  auto network = std::unique_ptr<nvinfer1::INetworkDefinition>(builder->createNetworkV2(flags));
  if (network == nullptr) return false;

  auto parser = std::unique_ptr<nvonnxparser::IParser>(
      nvonnxparser::createParser(*network, TensorRTLogger::instance()));
  if (parser == nullptr) return false;

  auto constructed = parser->parseFromFile(onnx_path.c_str(), 0);
  if (not constructed) return false;

  for (int i{}; i < network->getNbInputs(); ++i) {
    auto *input = network->getInput(i);
    STEPIT_ASSERT(input != nullptr, "TensorRT network input {} is null.", i);
    auto dims = input->getDimensions();
    STEPIT_ASSERT(not hasDynamicDim(dims),
                  "TensorRT input '{}' has dynamic shape {}. Use 'trtexec' for advanced dynamic-shape builds.",
                  input->getName(), dimsToString(dims));
  }

  auto config = std::unique_ptr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());
  if (config == nullptr) return false;
#if !TENSORRT_VERSION_AT_LEAST(10, 12, 0)
  if (use_fp16_) config->setFlag(nvinfer1::BuilderFlag::kFP16);
#endif

  auto plan = std::unique_ptr<nvinfer1::IHostMemory>{builder->buildSerializedNetwork(*network, *config)};
  if (plan == nullptr) return false;

  std::ofstream outfile(engine_path, std::ofstream::binary);
  if (not outfile.is_open()) return false;
  outfile.write(reinterpret_cast<const char *>(plan->data()), plan->size());
  return outfile.good();
}

bool TensorRTApi::createCudaGraph() {
  CudaDeviceGuard device_guard(device_id_);
  auto graph_stream = makeCudaStream();

  if (cudaStreamBeginCapture(graph_stream.get(), cudaStreamCaptureModeGlobal) != cudaSuccess) {
    STEPIT_WARNNT("During creating cuda graph: Failed to capture the cuda stream!");
    return false;
  }
  context_->enqueueV3(graph_stream.get());
  cudaGraph_t graph{nullptr};
  if (cudaStreamEndCapture(graph_stream.get(), &graph) != cudaSuccess) {
    STEPIT_WARNNT("During creating cuda graph: Failed to capture the cuda stream!");
    return false;
  }
  auto captured_graph = CudaGraphPtr(graph);
  cudaGraphExec_t instance{nullptr};
  if (cudaGraphInstantiate(&instance, captured_graph.get(), nullptr, nullptr, 0) != cudaSuccess) {
    STEPIT_WARNNT("During creating cuda graph: Failed to instantiate the cuda graph!");
    return false;
  }
  cuda_graph_    = std::move(captured_graph);
  cuda_instance_ = CudaGraphExecPtr(instance);
  return true;
}

void TensorRTApi::runInference() {
  CudaDeviceGuard device_guard(device_id_);
  if (cuda_instance_) {
    STEPIT_CUDA_CALL(cudaGraphLaunch, cuda_instance_.get(), cuda_stream_.get());
  } else {
    STEPIT_ASSERT(context_->enqueueV3(cuda_stream_.get()), "Failed to run inference!");
  }
  for (const auto &pair : recur_param_indices_) {
    STEPIT_CUDA_CALL(cudaMemcpyAsync, inputs_[pair.first].get(), outputs_[pair.second].get(),
                     tensorBytes(in_sizes_[pair.first]), cudaMemcpyDeviceToDevice, cuda_stream_.get());
  }
}

void TensorRTApi::clearState() {
  CudaDeviceGuard device_guard(device_id_);
  for (const auto &pair : recur_param_indices_) {
    STEPIT_CUDA_CALL(cudaMemsetAsync, inputs_[pair.first].get(), 0, tensorBytes(in_sizes_[pair.first]),
                     cuda_stream_.get());
  }
}

void TensorRTApi::synchronize() {
  CudaDeviceGuard device_guard(device_id_);
  STEPIT_CUDA_CALL(cudaStreamSynchronize, cuda_stream_.get());
}

void TensorRTApi::setInput(std::size_t idx, float *data) {
  CudaDeviceGuard device_guard(device_id_);
  STEPIT_CUDA_CALL(cudaMemcpyAsync, inputs_[idx].get(), data, tensorBytes(in_sizes_[idx]), cudaMemcpyHostToDevice,
                   cuda_stream_.get());
}

const float *TensorRTApi::getOutput(std::size_t idx) {
  CudaDeviceGuard device_guard(device_id_);
  STEPIT_CUDA_CALL(cudaMemcpyAsync, out_data_[idx].get(), outputs_[idx].get(), tensorBytes(out_sizes_[idx]),
                   cudaMemcpyDeviceToHost, cuda_stream_.get());
  synchronize();
  return out_data_[idx].get();
}

STEPIT_REGISTER_NNRTAPI(tensorrt, kDefPriority, NnrtApi::make<TensorRTApi>);
}  // namespace stepit

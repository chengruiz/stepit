#include <algorithm>
#include <iostream>
#include <numeric>

#include <stepit/nnrt/ascendcl.h>

#define STEPIT_ACL_CALL(api, ...)                                             \
  do {                                                                        \
    aclError ret = api(__VA_ARGS__);                                          \
    STEPIT_ASSERT(ret == ACL_SUCCESS, #api " failed (error code: {}).", ret); \
  } while (0)

namespace stepit {
void initializeACL() {
  static bool initialized{false};
  if (not initialized) {
    initialized = true;
    STEPIT_ACL_CALL(aclInit, nullptr);
  }
}

AscendCLApi::AscendCLApi(const std::string &path, const YAML::Node &config)
    : NnrtApi(addExtensionIfMissing(path, ".om"), config) {
  initializeACL();

  STEPIT_ACL_CALL(aclrtSetDevice, device_id_);
  STEPIT_ACL_CALL(aclrtCreateContext, &context_, device_id_);
  STEPIT_ACL_CALL(aclrtCreateStream, &stream_);
  STEPIT_ACL_CALL(aclmdlLoadFromFile, path_.c_str(), &model_id_);
  model_desc_ = aclmdlCreateDesc();
  STEPIT_ACL_CALL(aclmdlGetDesc, model_desc_, model_id_);
  STEPIT_ACL_CALL(aclrtGetRunMode, &run_mode_);

  in_dataset_ = aclmdlCreateDataset();
  num_in_     = aclmdlGetNumInputs(model_desc_);
  for (std::size_t i{}; i < num_in_; i++) {
    aclmdlIODims dims;
    STEPIT_ACL_CALL(aclmdlGetInputDims, model_desc_, i, &dims);
    std::vector<int64_t> shape;
    for (int j{}; j < dims.dimCount; ++j) shape.push_back(dims.dims[j]);
    in_shapes_.emplace_back(std::move(shape));
    in_bytes_.push_back(aclmdlGetInputSizeByIndex(model_desc_, i));
    in_sizes_.push_back(in_bytes_[i] / sizeof(float));
    in_names_.push_back(aclmdlGetInputNameByIndex(model_desc_, i));
    void *in_buffer = nullptr;
    STEPIT_ACL_CALL(aclrtMalloc, &in_buffer, in_bytes_[i], ACL_MEM_MALLOC_NORMAL_ONLY);
    in_buffers_.push_back(aclCreateDataBuffer(in_buffer, in_bytes_[i]));
    STEPIT_ACL_CALL(aclmdlAddDatasetBuffer, in_dataset_, in_buffers_[i]);
  }

  out_dataset_ = aclmdlCreateDataset();
  num_out_     = aclmdlGetNumOutputs(model_desc_);
  for (std::size_t i{}; i < num_out_; i++) {
    aclmdlIODims dims;
    STEPIT_ACL_CALL(aclmdlGetInputDims, model_desc_, i, &dims);
    std::vector<int64_t> shape;
    for (int j{}; j < dims.dimCount; ++j) shape.push_back(dims.dims[j]);
    out_shapes_.emplace_back(std::move(shape));
    out_bytes_.push_back(aclmdlGetOutputSizeByIndex(model_desc_, i));
    out_sizes_.push_back(out_bytes_[i] / sizeof(float));
    out_names_.push_back(aclmdlGetOutputNameByIndex(model_desc_, i));
    void *out_buffer = nullptr;
    STEPIT_ACL_CALL(aclrtMalloc, &out_buffer, out_bytes_[i], ACL_MEM_MALLOC_NORMAL_ONLY);
    out_buffers_.push_back(aclCreateDataBuffer(out_buffer, out_bytes_[i]));
    STEPIT_ACL_CALL(aclmdlAddDatasetBuffer, out_dataset_, out_buffers_[i]);
  }

  postInit();
}

AscendCLApi::~AscendCLApi() {
  if (in_dataset_ != nullptr) {
    for (std::size_t i{}; i < aclmdlGetDatasetNumBuffers(in_dataset_); i++) {
      void *data = aclGetDataBufferAddr(in_buffers_[i]);
      aclrtFree(data);
      aclDestroyDataBuffer(in_buffers_[i]);
    }
    aclmdlDestroyDataset(in_dataset_);
  }
  if (out_dataset_ != nullptr) {
    for (std::size_t i{}; i < aclmdlGetDatasetNumBuffers(out_dataset_); i++) {
      void *data = aclGetDataBufferAddr(out_buffers_[i]);
      aclrtFree(data);
      aclDestroyDataBuffer(out_buffers_[i]);
    }
    aclmdlDestroyDataset(out_dataset_);
  }

  aclmdlUnload(model_id_);
  aclmdlDestroyDesc(model_desc_);
  if (stream_ != nullptr) aclrtDestroyStream(stream_);
  if (context_ != nullptr) aclrtDestroyContext(context_);
  aclrtResetDevice(device_id_);
  aclFinalize();
}

void AscendCLApi::runInference() {
  STEPIT_ACL_CALL(aclrtSetCurrentContext, context_);
  STEPIT_ACL_CALL(aclmdlExecuteAsync, model_id_, in_dataset_, out_dataset_, stream_);
  for (const auto &pair : recur_param_indices_) {
    STEPIT_ACL_CALL(aclrtMemcpyAsync, aclGetDataBufferAddr(in_buffers_[pair.first]), out_bytes_[pair.second],
                    aclGetDataBufferAddr(out_buffers_[pair.second]), out_bytes_[pair.second],
                    ACL_MEMCPY_DEVICE_TO_DEVICE, stream_);
  }
}

void AscendCLApi::clearState() {
  STEPIT_ACL_CALL(aclrtSetCurrentContext, context_);
  for (const auto &pair : recur_param_indices_) {
    STEPIT_ACL_CALL(aclrtMemsetAsync, aclGetDataBufferAddr(in_buffers_[pair.first]), in_bytes_[pair.first], 0,
                    in_bytes_[pair.first], stream_);
  }
}

void AscendCLApi::setInput(std::size_t idx, float *data) {
  aclrtMemcpyKind kind;
  if (run_mode_ == ACL_DEVICE) {
    kind = ACL_MEMCPY_DEVICE_TO_DEVICE;
  } else {
    kind = ACL_MEMCPY_HOST_TO_DEVICE;
  }
  STEPIT_ACL_CALL(aclrtSetCurrentContext, context_);
  STEPIT_ACL_CALL(aclrtMemcpyAsync, aclGetDataBufferAddr(in_buffers_[idx]), in_bytes_[idx], data, in_bytes_[idx], kind,
                  stream_);
}

const float *AscendCLApi::getOutput(std::size_t idx) {
  STEPIT_ACL_CALL(aclrtSetCurrentContext, context_);
  synchronize();
  aclDataBuffer *dataBuffer = aclmdlGetDatasetBuffer(out_dataset_, idx);
  void *data                = aclGetDataBufferAddr(dataBuffer);
  if (run_mode_ == ACL_DEVICE) {
    return reinterpret_cast<float *>(data);
  }
  // FIXME: verify this
  uint32_t len        = aclGetDataBufferSizeV2(dataBuffer);
  void *out_host_data = nullptr;
  float *out_data     = nullptr;
  STEPIT_ACL_CALL(aclrtMallocHost, &out_host_data, len);
  STEPIT_ACL_CALL(aclrtMemcpy, out_host_data, len, data, len, ACL_MEMCPY_DEVICE_TO_HOST);
  out_data = reinterpret_cast<float *>(out_host_data);
  STEPIT_ACL_CALL(aclrtFreeHost, out_host_data);
  return out_data;
}

void AscendCLApi::synchronize() { STEPIT_ACL_CALL(aclrtSynchronizeStream, stream_); }

STEPIT_REGISTER_NNRTAPI(ascendcl, kDefPriority, NnrtApi::make<AscendCLApi>);
}  // namespace stepit

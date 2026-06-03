#include <cstring>

#include <stepit/nnrt/onnxruntime.h>

namespace stepit {
OnnxRt::OnnxRt(const std::string &path, const yml::Node &config) : Nnrt(addExtensionIfMissing(path, ".onnx"), config) {
  env_         = Ort::Env(ORT_LOGGING_LEVEL_WARNING, path_.c_str());
  memory_info_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  Ort::SessionOptions opts;
  opts.SetInterOpNumThreads(1);
  opts.SetIntraOpNumThreads(1);
  core_       = std::make_unique<Ort::Session>(env_, path_.c_str(), opts);
  num_in_     = core_->GetInputCount();
  num_out_    = core_->GetOutputCount();
  io_binding_ = std::make_unique<Ort::IoBinding>(*core_);

  for (std::size_t i{}; i < num_in_; ++i) {
    auto type_info   = core_->GetInputTypeInfo(i);
    auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
    auto shape       = tensor_info.GetShape();
    for (auto &dim : shape) {
      if (dim == -1) dim = 1;
    }
    auto dtype            = mapOnnxDtype(tensor_info.GetElementType());
    int64_t size          = product(shape);
    auto name             = core_->GetInputNameAllocated(i, allocator_);
    std::size_t byte_size = static_cast<std::size_t>(size) * dataTypeSize(dtype);

    addInput(name.get(), std::move(shape), size, dtype);
    in_data_.emplace_back(byte_size, 0);
    in_tensors_.push_back(createTensor(in_data_[i].data(), byte_size, in_shapes_[i], dtype));
    io_binding_->BindInput(in_names_[i].c_str(), in_tensors_[i]);
  }

  for (std::size_t i{}; i < num_out_; ++i) {
    auto type_info   = core_->GetOutputTypeInfo(i);
    auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
    auto shape       = tensor_info.GetShape();
    for (auto &dim : shape) {
      if (dim == -1) dim = 1;
    }
    auto dtype            = mapOnnxDtype(tensor_info.GetElementType());
    int64_t size          = product(shape);
    auto name             = core_->GetOutputNameAllocated(i, allocator_);
    std::size_t byte_size = static_cast<std::size_t>(size) * dataTypeSize(dtype);

    addOutput(name.get(), std::move(shape), size, dtype);
    out_data_.emplace_back(byte_size, 0);
    out_tensors_.push_back(createTensor(out_data_[i].data(), byte_size, out_shapes_[i], dtype));
    io_binding_->BindOutput(out_names_[i].c_str(), out_tensors_[i]);
  }

  postInit();
}

void OnnxRt::runInference() {
  core_->Run(run_options_, *io_binding_);
  for (const auto &pair : recur_param_indices_) {
    std::size_t bytes = static_cast<std::size_t>(in_sizes_[pair.first]) * dataTypeSize(in_dtypes_[pair.first]);
    std::memcpy(in_data_[pair.first].data(), out_data_[pair.second].data(), bytes);
  }
}

void OnnxRt::clearState() {
  for (const auto &pair : recur_param_indices_) {
    std::fill(in_data_[pair.first].begin(), in_data_[pair.first].end(), 0);
  }
}

void OnnxRt::setInput(std::size_t idx, const void *data) {
  std::size_t bytes = static_cast<std::size_t>(in_sizes_[idx]) * dataTypeSize(in_dtypes_[idx]);
  std::memcpy(in_data_[idx].data(), data, bytes);
}

const void *OnnxRt::getOutput(std::size_t idx) { return out_data_[idx].data(); }

DataType OnnxRt::mapOnnxDtype(ONNXTensorElementDataType onnx_type) {
  switch (onnx_type) {
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT:
      return DataType::kFloat32;
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32:
      return DataType::kInt32;
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64:
      return DataType::kInt64;
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_BOOL:
      return DataType::kBool;
    default:
      STEPIT_THROW("Unsupported ONNX tensor element type: {}.", static_cast<int>(onnx_type));
  }
}

Ort::Value OnnxRt::createTensor(void *data, std::size_t byte_size, const std::vector<int64_t> &shape, DataType dtype) {
  switch (dtype) {
    case DataType::kFloat32:
      return Ort::Value::CreateTensor<float>(memory_info_, static_cast<float *>(data), byte_size / sizeof(float),
                                             shape.data(), shape.size());
    case DataType::kInt32:
      return Ort::Value::CreateTensor<int32_t>(memory_info_, static_cast<int32_t *>(data), byte_size / sizeof(int32_t),
                                               shape.data(), shape.size());
    case DataType::kInt64:
      return Ort::Value::CreateTensor<int64_t>(memory_info_, static_cast<int64_t *>(data), byte_size / sizeof(int64_t),
                                               shape.data(), shape.size());
    case DataType::kBool:
      return Ort::Value::CreateTensor<bool>(memory_info_, static_cast<bool *>(data), byte_size / sizeof(bool),
                                            shape.data(), shape.size());
  }
  STEPIT_THROW("Unsupported DataType in createTensor.");
}

STEPIT_REGISTER_NNRT(onnxruntime, kDefPriority - 1, Nnrt::make<OnnxRt>);
}  // namespace stepit

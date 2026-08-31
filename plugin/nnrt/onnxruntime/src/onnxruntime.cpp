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
  in_data_.reserve(num_in_);
  in_tensors_.reserve(num_in_);
  out_data_.reserve(num_out_);
  out_tensors_.reserve(num_out_);

  for (std::size_t i{}; i < num_in_; ++i) {
    auto type_info   = core_->GetInputTypeInfo(i);
    auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
    auto shape       = tensor_info.GetShape();
    for (auto &dim : shape) {
      if (dim == -1) dim = 1;
    }
    auto dtype   = mapOnnxDtype(tensor_info.GetElementType());
    int64_t size = product(shape);
    auto name    = core_->GetInputNameAllocated(i, allocator_);

    addInput(name.get(), std::move(shape), dtype, size);
    in_data_.emplace_back(dtype, static_cast<std::size_t>(size));
    auto &data = in_data_.back();
    if (data.byteSize() != 0) std::memset(data.data(), 0, data.byteSize());
    in_tensors_.push_back(createTensor(data, in_shapes_[i]));
    io_binding_->BindInput(in_names_[i].c_str(), in_tensors_[i]);
  }

  for (std::size_t i{}; i < num_out_; ++i) {
    auto type_info   = core_->GetOutputTypeInfo(i);
    auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
    auto shape       = tensor_info.GetShape();
    for (auto &dim : shape) {
      if (dim == -1) dim = 1;
    }
    auto dtype   = mapOnnxDtype(tensor_info.GetElementType());
    int64_t size = product(shape);
    auto name    = core_->GetOutputNameAllocated(i, allocator_);

    addOutput(name.get(), std::move(shape), dtype, size);
    out_data_.emplace_back(dtype, static_cast<std::size_t>(size));
    auto &data = out_data_.back();
    if (data.byteSize() != 0) std::memset(data.data(), 0, data.byteSize());
    out_tensors_.push_back(createTensor(data, out_shapes_[i]));
    io_binding_->BindOutput(out_names_[i].c_str(), out_tensors_[i]);
  }

  postInit();
}

void OnnxRt::runInference() {
  core_->Run(run_options_, *io_binding_);
  for (const auto &pair : recur_param_indices_) {
    auto &input        = in_data_[pair.first];
    const auto &output = out_data_[pair.second];
    std::memcpy(input.data(), output.data(), input.byteSize());
  }
}

void OnnxRt::clearState() {
  for (const auto &pair : recur_param_indices_) {
    auto &input = in_data_[pair.first];
    if (input.byteSize() != 0) std::memset(input.data(), 0, input.byteSize());
  }
}

void OnnxRt::setInput(std::size_t idx, const void *data) {
  std::memcpy(in_data_[idx].data(), data, in_data_[idx].byteSize());
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

Ort::Value OnnxRt::createTensor(TypedBuffer &buffer, const std::vector<int64_t> &shape) {
  switch (buffer.dataType()) {
    case DataType::kUndefined:
      STEPIT_THROW("Cannot create an ONNX tensor with undefined data type.");
    case DataType::kFloat32:
      return Ort::Value::CreateTensor<float>(memory_info_, static_cast<float *>(static_cast<void *>(buffer.data())),
                                             buffer.size(), shape.data(), shape.size());
    case DataType::kInt32:
      return Ort::Value::CreateTensor<int32_t>(memory_info_, static_cast<int32_t *>(static_cast<void *>(buffer.data())),
                                               buffer.size(), shape.data(), shape.size());
    case DataType::kInt64:
      return Ort::Value::CreateTensor<int64_t>(memory_info_, static_cast<int64_t *>(static_cast<void *>(buffer.data())),
                                               buffer.size(), shape.data(), shape.size());
    case DataType::kBool:
      return Ort::Value::CreateTensor<bool>(memory_info_, static_cast<bool *>(static_cast<void *>(buffer.data())),
                                            buffer.size(), shape.data(), shape.size());
  }
  STEPIT_THROW("Unsupported DataType in createTensor.");
}

STEPIT_REGISTER_NNRT(onnxruntime, kDefPriority - 1, Nnrt::make<OnnxRt>);
}  // namespace stepit

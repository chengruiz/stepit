#include <cstring>
#include <numeric>
#include <stdexcept>

#include <stepit/nnrt/torchjit.h>

namespace stepit {
DataType TorchJit::mapTorchDtype(torch::ScalarType scalar_type) {
  switch (scalar_type) {
    case torch::kFloat32:
      return DataType::kFloat32;
    case torch::kInt32:
      return DataType::kInt32;
    case torch::kInt64:
      return DataType::kInt64;
    case torch::kBool:
      return DataType::kBool;
    default:
      STEPIT_THROW("Unsupported torch ScalarType: {}.", static_cast<int>(scalar_type));
  }
}

torch::ScalarType TorchJit::toTorchDtype(DataType dtype) {
  switch (dtype) {
    case DataType::kFloat32:
      return torch::kFloat32;
    case DataType::kInt32:
      return torch::kInt32;
    case DataType::kInt64:
      return torch::kInt64;
    case DataType::kBool:
      return torch::kBool;
  }
  return torch::kFloat32;
}

TorchJit::TorchJit(const std::string &path, const yml::Node &config)
    : Nnrt(addExtensionIfMissing(path, ".pt"), config) {
  module_ = torch::jit::load(path_, torch::kCPU);
  module_.eval();

  initInputSpec();
  initOutputSpec();
  postInit();
}

void TorchJit::runInference() {
  torch::NoGradGuard no_grad;

  std::vector<torch::jit::IValue> in_tensors;
  in_tensors.reserve(num_in_);
  for (std::size_t i{}; i < num_in_; ++i) {
    in_tensors.emplace_back(torch::from_blob(in_data_[i].data(), in_shapes_[i], toTorchDtype(in_dtypes_[i])));
  }

  auto out_tensors = extractOutputTensors(module_.forward(in_tensors));
  STEPIT_ASSERT(out_tensors.size() == num_out_, "TorchJit output count mismatch: expected {}, got {}.", num_out_,
                out_tensors.size());
  for (std::size_t i{}; i < num_out_; ++i) {
    out_tensors[i]   = normalizeTensor(out_tensors[i]);
    int64_t out_size = out_tensors[i].numel();
    STEPIT_ASSERT(out_size == out_sizes_[i], "TorchJit output '{}' size mismatch: expected {}, got {}.", out_names_[i],
                  out_sizes_[i], out_size);
    std::size_t bytes = static_cast<std::size_t>(out_size) * dataTypeSize(out_dtypes_[i]);
    std::memcpy(out_data_[i].data(), out_tensors[i].data_ptr(), bytes);
  }

  for (const auto &pair : recur_param_indices_) {
    std::size_t bytes = static_cast<std::size_t>(in_sizes_[pair.first]) * dataTypeSize(in_dtypes_[pair.first]);
    std::memcpy(in_data_[pair.first].data(), out_data_[pair.second].data(), bytes);
  }
}

void TorchJit::clearState() {
  for (const auto &pair : recur_param_indices_) {
    std::fill(in_data_[pair.first].begin(), in_data_[pair.first].end(), 0);
  }
}

void TorchJit::setInput(std::size_t idx, const void *data) {
  std::size_t bytes = static_cast<std::size_t>(in_sizes_[idx]) * dataTypeSize(in_dtypes_[idx]);
  std::memcpy(in_data_[idx].data(), data, bytes);
}

const void *TorchJit::getOutput(std::size_t idx) { return out_data_[idx].data(); }

torch::Tensor TorchJit::normalizeTensor(const torch::Tensor &tensor) {
  auto normalized = tensor.detach().to(torch::kCPU).contiguous();
  if (normalized.sizes().empty()) {
    normalized = normalized.reshape({1});
  }
  return normalized;
}

std::vector<torch::Tensor> TorchJit::extractOutputTensors(const torch::jit::IValue &output) {
  if (output.isTensor()) {
    return {output.toTensor()};
  }

  std::vector<torch::Tensor> tensors;
  if (output.isTuple()) {
    for (const auto &item : output.toTuple()->elements()) {
      auto nested = extractOutputTensors(item);
      tensors.insert(tensors.end(), nested.begin(), nested.end());
    }
    return tensors;
  }

  if (output.isTensorList()) {
    auto list = output.toTensorVector();
    tensors.insert(tensors.end(), list.begin(), list.end());
    return tensors;
  }

  if (output.isList()) {
    for (const auto &item : output.toListRef()) {
      auto nested = extractOutputTensors(item);
      tensors.insert(tensors.end(), nested.begin(), nested.end());
    }
    return tensors;
  }

  throw std::runtime_error("Unsupported TorchScript output type.");
}

void TorchJit::initInputSpec() {
  auto shapes = config_["input_shapes"].as<std::vector<std::vector<int64_t>>>();
  num_in_     = shapes.size();
  STEPIT_ASSERT(num_in_ > 0, "TorchJit requires 'input_shape' or 'input_shapes' in model config.");

  std::vector<std::string> dtype_strs;
  if (config_.has("input_dtypes")) {
    dtype_strs = config_["input_dtypes"].as<std::vector<std::string>>();
    STEPIT_ASSERT(dtype_strs.size() == num_in_, "'input_dtypes' count mismatch: expected {}, got {}.", num_in_,
                  dtype_strs.size());
  }

  in_data_.reserve(num_in_);
  for (std::size_t i{}; i < num_in_; ++i) {
    STEPIT_ASSERT(not shapes[i].empty(), "TorchJit input shape cannot be empty at index {}.", i);
    for (auto dim : shapes[i]) {
      STEPIT_ASSERT(dim > 0, "TorchJit input shape must be static and positive at index {} with dimension {}.", i, dim);
    }
    auto in_size   = product(shapes[i]);
    DataType dtype = DataType::kFloat32;
    if (not dtype_strs.empty()) {
      const auto &s = dtype_strs[i];
      if (s == "float32")
        dtype = DataType::kFloat32;
      else if (s == "int32")
        dtype = DataType::kInt32;
      else if (s == "int64")
        dtype = DataType::kInt64;
      else if (s == "bool")
        dtype = DataType::kBool;
      else
        STEPIT_THROW("Unknown dtype string '{}' in 'input_dtypes'.", s);
    }
    addInput("input" + std::to_string(i), std::move(shapes[i]), in_size, dtype);
    in_data_.emplace_back(static_cast<std::size_t>(in_size) * dataTypeSize(dtype), 0);
  }
}

void TorchJit::initOutputSpec() {
  torch::NoGradGuard no_grad;

  std::vector<torch::jit::IValue> in_tensors;
  in_tensors.reserve(num_in_);
  for (std::size_t i{}; i < num_in_; ++i) {
    in_tensors.emplace_back(torch::from_blob(in_data_[i].data(), in_shapes_[i], toTorchDtype(in_dtypes_[i])));
  }

  auto out_tensors = extractOutputTensors(module_.forward(in_tensors));
  STEPIT_ASSERT(not out_tensors.empty(), "TorchJit model has no tensor outputs.");

  num_out_ = out_tensors.size();
  out_data_.reserve(num_out_);

  for (std::size_t i{}; i < num_out_; ++i) {
    out_tensors[i] = normalizeTensor(out_tensors[i]);
    auto dtype     = mapTorchDtype(out_tensors[i].scalar_type());

    auto shape        = std::vector<int64_t>(out_tensors[i].sizes().begin(), out_tensors[i].sizes().end());
    auto size         = static_cast<int64_t>(out_tensors[i].numel());
    std::size_t bytes = static_cast<std::size_t>(size) * dataTypeSize(dtype);

    addOutput("output" + std::to_string(i), std::move(shape), size, dtype);
    out_data_.emplace_back(bytes, 0);
    std::memcpy(out_data_[i].data(), out_tensors[i].data_ptr(), bytes);
  }
}

STEPIT_REGISTER_NNRT(torchjit, kDefPriority, Nnrt::make<TorchJit>);
}  // namespace stepit

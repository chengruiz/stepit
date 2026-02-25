#include <cstring>
#include <numeric>
#include <stdexcept>

#include <stepit/nnrt/torchjit.h>

namespace stepit {
TorchJitApi::TorchJitApi(const std::string &path, const YAML::Node &config)
    : NnrtApi(addExtensionIfMissing(path, ".pt"), config) {
  module_ = torch::jit::load(path_, torch::kCPU);
  module_.eval();

  initInputSpec();
  initOutputSpec();
  postInit();
}

void TorchJitApi::runInference() {
  torch::NoGradGuard no_grad;

  std::vector<torch::jit::IValue> in_tensors;
  in_tensors.reserve(num_in_);
  for (std::size_t i{}; i < num_in_; ++i) {
    in_tensors.emplace_back(torch::from_blob(in_data_[i].data(), in_shapes_[i], torch::kFloat32));
  }

  auto out_tensors = extractOutputTensors(module_.forward(in_tensors));
  STEPIT_ASSERT(out_tensors.size() == num_out_, "TorchJit output count mismatch: expected {}, got {}.", num_out_,
                out_tensors.size());
  for (std::size_t i{}; i < num_out_; ++i) {
    out_tensors[i]   = normalizeTensor(out_tensors[i]);
    int64_t out_size = out_tensors[i].numel();
    STEPIT_ASSERT(out_size == out_sizes_[i], "TorchJit output '{}' size mismatch: expected {}, got {}.", out_names_[i],
                  out_sizes_[i], out_size);
    std::memcpy(out_data_[i].data(), out_tensors[i].data_ptr<float>(), out_size * sizeof(float));
  }

  for (const auto &pair : recur_param_indices_) {
    std::copy_n(out_data_[pair.second].data(), in_sizes_[pair.first], in_data_[pair.first].data());
  }
}

void TorchJitApi::clearState() {
  for (const auto &pair : recur_param_indices_) {
    std::fill(in_data_[pair.first].begin(), in_data_[pair.first].end(), 0.0F);
  }
}

void TorchJitApi::setInput(std::size_t idx, float *data) { std::copy_n(data, in_sizes_[idx], in_data_[idx].data()); }

const float *TorchJitApi::getOutput(std::size_t idx) { return out_data_[idx].data(); }

torch::Tensor TorchJitApi::normalizeTensor(const torch::Tensor &tensor) {
  auto normalized = tensor.detach().to(torch::kCPU).to(torch::kFloat32).contiguous();
  if (normalized.sizes().empty()) {
    normalized = normalized.reshape({1});
  }
  return normalized;
}

std::vector<torch::Tensor> TorchJitApi::extractOutputTensors(const torch::jit::IValue &output) {
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

void TorchJitApi::initInputSpec() {
  in_shapes_ = config_["input_shapes"].as<std::vector<std::vector<int64_t>>>();
  num_in_    = in_shapes_.size();
  in_shapes_.resize(num_in_);
  STEPIT_ASSERT(num_in_ > 0, "TorchJit requires 'input_shape' or 'input_shapes' in model config.");

  in_sizes_.reserve(num_in_);
  in_names_.reserve(num_in_);
  in_data_.reserve(num_in_);
  for (std::size_t i{}; i < num_in_; ++i) {
    STEPIT_ASSERT(not in_shapes_[i].empty(), "TorchJit input shape cannot be empty at index {}.", i);
    for (auto dim : in_shapes_[i]) {
      STEPIT_ASSERT(dim > 0, "TorchJit input shape must be static and positive at index {} with dimension {}.", i, dim);
    }
    auto in_size = product(in_shapes_[i]);
    in_sizes_.push_back(in_size);
    in_names_.push_back("input" + std::to_string(i));
    in_data_.emplace_back(in_size, 0.0F);
  }
}

void TorchJitApi::initOutputSpec() {
  torch::NoGradGuard no_grad;

  std::vector<torch::jit::IValue> in_tensors;
  in_tensors.reserve(num_in_);
  for (std::size_t i{}; i < num_in_; ++i) {
    in_tensors.emplace_back(torch::from_blob(in_data_[i].data(), in_shapes_[i], torch::kFloat32));
  }

  auto out_tensors = extractOutputTensors(module_.forward(in_tensors));
  STEPIT_ASSERT(not out_tensors.empty(), "TorchJit model has no tensor outputs.");

  num_out_ = out_tensors.size();
  out_shapes_.reserve(num_out_);
  out_sizes_.reserve(num_out_);
  out_names_.reserve(num_out_);
  out_data_.reserve(num_out_);

  for (std::size_t i{}; i < num_out_; ++i) {
    out_tensors[i] = normalizeTensor(out_tensors[i]);

    auto shape = std::vector<int64_t>(out_tensors[i].sizes().begin(), out_tensors[i].sizes().end());
    auto size  = static_cast<int64_t>(out_tensors[i].numel());

    out_shapes_.push_back(std::move(shape));
    out_sizes_.push_back(size);
    out_names_.push_back("output" + std::to_string(i));
    out_data_.emplace_back(size, 0.0F);
    std::memcpy(out_data_[i].data(), out_tensors[i].data_ptr<float>(), static_cast<std::size_t>(size) * sizeof(float));
  }
}

STEPIT_REGISTER_NNRTAPI(torchjit, kDefPriority, NnrtApi::make<TorchJitApi>);
}  // namespace stepit

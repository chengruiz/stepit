#include <stepit/nnrt/onnxruntime.h>

namespace stepit {
OnnxrtApi::OnnxrtApi(const std::string &path, const YAML::Node &config)
    : NnrtApi(path, config), env_(ORT_LOGGING_LEVEL_WARNING, path.c_str()) {
  memory_info_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  Ort::SessionOptions opts;
  opts.SetInterOpNumThreads(1);
  opts.SetIntraOpNumThreads(1);
  core_       = std::make_unique<Ort::Session>(env_, path.c_str(), opts);
  num_in_     = core_->GetInputCount();
  num_out_    = core_->GetOutputCount();
  io_binding_ = std::make_unique<Ort::IoBinding>(*core_);

  for (std::size_t i{}; i < num_in_; ++i) {
    auto shape   = getShapeFromTypeInfo(core_->GetInputTypeInfo(i));
    int64_t size = product(shape);
    auto name    = core_->GetInputNameAllocated(i, allocator_);

    in_shapes_.emplace_back(std::move(shape));
    in_sizes_.push_back(size);
    in_names_.emplace_back(name.get());
    in_data_.emplace_back(size, 0.0F);
    in_tensors_.push_back(createTensor(in_data_[i].data(), size, in_shapes_[i]));
    io_binding_->BindInput(in_names_[i].c_str(), in_tensors_[i]);
  }

  for (std::size_t i{}; i < num_out_; ++i) {
    auto shape   = getShapeFromTypeInfo(core_->GetOutputTypeInfo(i));
    int64_t size = product(shape);
    auto name    = core_->GetOutputNameAllocated(i, allocator_);

    out_shapes_.emplace_back(std::move(shape));
    out_data_.emplace_back(size, 0.0F);
    out_sizes_.push_back(size);
    out_names_.emplace_back(name.get());
    out_tensors_.push_back(createTensor(out_data_[i].data(), size, out_shapes_[i]));
    io_binding_->BindOutput(out_names_[i].c_str(), out_tensors_[i]);
  }

  postInit();
}

void OnnxrtApi::runInference() {
  core_->Run(run_options_, *io_binding_);
  for (const auto &pair : recur_param_indices_) {
    std::copy_n(out_data_[pair.second].data(), in_sizes_[pair.first], in_data_[pair.first].data());
  }
}

void OnnxrtApi::clearState() {
  for (const auto &pair : recur_param_indices_) {
    std::fill(in_data_[pair.first].begin(), in_data_[pair.first].end(), 0.0F);
  }
}

void OnnxrtApi::setInput(std::size_t idx, float *data) {  // copy data to input buffer
  std::copy_n(data, in_sizes_[idx], in_data_[idx].data());
}

const float *OnnxrtApi::getOutput(std::size_t idx) { return out_data_[idx].data(); }

std::vector<int64_t> OnnxrtApi::getShapeFromTypeInfo(const Ort::TypeInfo &type_info) {
  auto info  = type_info.GetTensorTypeAndShapeInfo();
  auto shape = info.GetShape();
  for (auto &dim : shape) {
    if (dim == -1) dim = 1;
  }
  return shape;
}

STEPIT_REGISTER_NNRTAPI(onnxruntime, kDefPriority, NnrtApi::make<OnnxrtApi>);
}  // namespace stepit

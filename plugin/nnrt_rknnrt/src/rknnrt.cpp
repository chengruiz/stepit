#include <stepit/nnrt/rknnrt.h>

#define LQC_RKNN_CALL(api, ...)                                     \
  do {                                                              \
    int ret = api(__VA_ARGS__);                                     \
    STEPIT_ASSERT(ret >= 0, #api " failed (error code: {}).", ret); \
  } while (0)

namespace stepit {
std::vector<int64_t> getShapeFromDimArray(uint32_t dims[RKNN_MAX_DIMS]) {
  std::vector<int64_t> shape;
  for (int i{}; i < RKNN_MAX_DIMS; ++i) {
    if (dims[i] == 0) break;
    shape.emplace_back(dims[i]);
  }
  return shape;
}

RknnrtApi::RknnrtApi(const std::string &path, const YAML::Node &config)
    : NnrtApi(addExtensionIfMissing(path, ".rknn"), config) {
  rknn_sdk_version version;
  rknn_input_output_num io_num;
  LQC_RKNN_CALL(rknn_init, &ctx_, (void *)path_.c_str(), 0, 0, nullptr);
  LQC_RKNN_CALL(rknn_query, ctx_, RKNN_QUERY_SDK_VERSION, &version, sizeof(version));
  STEPIT_LOGNT("RKNN api version {}, driver version {}.", version.api_version, version.drv_version);
  LQC_RKNN_CALL(rknn_query, ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
  num_in_  = io_num.n_input;
  num_out_ = io_num.n_output;

  rknn_tensor_attr attr;
  for (uint32_t i{}; i < num_in_; i++) {
    memset(&attr, 0, sizeof(rknn_tensor_attr));
    attr.index = i;
    LQC_RKNN_CALL(rknn_query, ctx_, RKNN_QUERY_NATIVE_INPUT_ATTR, &attr, sizeof(rknn_tensor_attr));
    in_shapes_.push_back(getShapeFromDimArray(attr.dims));
    in_sizes_.push_back(attr.n_elems);
    in_names_.emplace_back(attr.name);

    uint32_t n_bytes = attr.n_elems * sizeof(float);
    in_bytes_.push_back(n_bytes);
    auto *mem = rknn_create_mem(ctx_, n_bytes);
    attr.type = RKNN_TENSOR_FLOAT32;
    attr.size = attr.size_with_stride = n_bytes;
    LQC_RKNN_CALL(rknn_set_io_mem, ctx_, mem, &attr);
    in_mems_.push_back(mem);
  }

  for (uint32_t i{}; i < num_out_; i++) {
    memset(&attr, 0, sizeof(rknn_tensor_attr));
    attr.index = i;
    LQC_RKNN_CALL(rknn_query, ctx_, RKNN_QUERY_NATIVE_OUTPUT_ATTR, &attr, sizeof(rknn_tensor_attr));
    out_shapes_.push_back(getShapeFromDimArray(attr.dims));
    out_sizes_.push_back(attr.n_elems);
    out_names_.emplace_back(attr.name);

    uint32_t n_bytes = attr.n_elems * sizeof(float);
    out_bytes_.push_back(attr.n_elems * sizeof(float));
    auto *mem = rknn_create_mem(ctx_, n_bytes);
    attr.type = RKNN_TENSOR_FLOAT32;
    attr.size = attr.size_with_stride = n_bytes;
    LQC_RKNN_CALL(rknn_set_io_mem, ctx_, mem, &attr);
    out_mems_.push_back(mem);
  }

  postInit();
}

RknnrtApi::~RknnrtApi() {
  for (auto &&mem : in_mems_) rknn_destroy_mem(ctx_, mem);
  for (auto &&mem : out_mems_) rknn_destroy_mem(ctx_, mem);
  rknn_destroy(ctx_);
}

void RknnrtApi::runInference() {
  LQC_RKNN_CALL(rknn_run, ctx_, nullptr);
  for (const auto &pair : recur_param_indices_) {
    std::copy_n(static_cast<float *>(out_mems_[pair.second]->virt_addr), in_sizes_[pair.first],
                static_cast<float *>(in_mems_[pair.first]->virt_addr));
  }
}

void RknnrtApi::clearState() {
  for (const auto &pair : recur_param_indices_) {
    std::fill_n(static_cast<float *>(in_mems_[pair.first]->virt_addr), in_sizes_[pair.first], 0.0F);
  }
}

void RknnrtApi::setInput(std::size_t idx, float *data) {
  std::copy_n(data, in_sizes_[idx], static_cast<float *>(in_mems_[idx]->virt_addr));
}

const float *RknnrtApi::getOutput(std::size_t idx) { return static_cast<float *>(out_mems_[idx]->virt_addr); }

STEPIT_REGISTER_NNRTAPI(rknnrt, kDefPriority, NnrtApi::make<RknnrtApi>);
}  // namespace stepit

#ifndef STEPIT_NNRT_RKNNRT_H_
#define STEPIT_NNRT_RKNNRT_H_

#include <string>
#include <vector>

#include <rknn_api.h>

#include <stepit/nnrt/nnrt.h>

namespace stepit {
class RknnrtApi : public NnrtApi {
 public:
  explicit RknnrtApi(const std::string &path, const yml::Node &config);
  ~RknnrtApi() override;

  void runInference() override;
  void clearState() override;

  using NnrtApi::setInput;
  void setInput(std::size_t idx, float *data) override;
  using NnrtApi::getOutput;
  const float *getOutput(std::size_t idx) override;

 private:
  rknn_context ctx_{};
  std::vector<uint32_t> in_bytes_, out_bytes_;
  std::vector<rknn_tensor_mem *> in_mems_, out_mems_;
};
}  // namespace stepit

#endif  // STEPIT_NNRT_RKNNRT_H_

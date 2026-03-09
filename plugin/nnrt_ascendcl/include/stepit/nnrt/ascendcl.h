#ifndef STEPIT_NNRT_ASCENDCL_H_
#define STEPIT_NNRT_ASCENDCL_H_

#include <string>
#include <vector>

#include <acl/acl.h>

#include <stepit/nnrt/nnrt.h>

namespace stepit {
class AscendCLApi : public NnrtApi {
 public:
  AscendCLApi(const std::string &path, const yml::Node &config);
  virtual ~AscendCLApi();

  void clearState() override;
  void runInference() override;

  using NnrtApi::setInput;
  void setInput(std::size_t idx, float *data) override;
  using NnrtApi::getOutput;
  const float *getOutput(std::size_t idx) override;

 private:
  void synchronize() override;

  aclrtContext context_;
  aclrtStream stream_;

  int32_t device_id_{};
  uint32_t model_id_{};
  aclmdlDesc *model_desc_{nullptr};
  aclrtRunMode run_mode_;
  std::vector<int64_t> in_bytes_, out_bytes_;
  aclmdlDataset *in_dataset_{nullptr}, *out_dataset_{nullptr};
  std::vector<aclDataBuffer *> in_buffers_, out_buffers_;
};
}  // namespace stepit

#endif  // STEPIT_NNRT_ASCENDCL_H_

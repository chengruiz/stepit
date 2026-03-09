#ifndef STEPIT_NNRT_H_
#define STEPIT_NNRT_H_

#include <string>
#include <vector>

#include <stepit/registry.h>

namespace stepit {
class NnrtApi : public Interface<NnrtApi, const std::string & /* path */, const yml::Node & /* config */> {
 public:
  NnrtApi(const std::string &path, const yml::Node &config);
  NnrtApi(const NnrtApi &)            = delete;
  NnrtApi &operator=(const NnrtApi &) = delete;

  virtual void runInference() = 0;
  virtual void clearState() {}
  void warmup(int iterations = 10);
  void printInfo() const;

  std::size_t getNumInputs() const { return num_in_; }
  const std::vector<std::string> &getInputNames() const { return in_names_; }
  const std::string &getInputName(std::size_t idx) const { return in_names_[idx]; }
  bool hasInput(const std::string &name) const { return getInputIdx(name) != static_cast<std::size_t>(-1); }
  std::size_t getInputIdx(const std::string &name, bool assert = false) const;
  void setInput(const std::string &name, float *data) { setInput(getInputIdx(name, true), data); }
  virtual void setInput(std::size_t idx, float *data) = 0;
  std::size_t getInputSize(const std::string &name) const { return getInputSize(getInputIdx(name, true)); }
  std::size_t getInputSize(std::size_t idx) const { return in_sizes_[idx]; }
  bool isInputRecurrent(const std::string &name) const { return isInputRecurrent(getInputIdx(name, true)); }
  bool isInputRecurrent(std::size_t idx) const { return in_recur_[idx]; }

  std::size_t getNumOutputs() const { return num_out_; }
  const std::vector<std::string> &getOutputNames() const { return out_names_; }
  const std::string &getOutputName(std::size_t idx) const { return out_names_[idx]; }
  bool hasOutput(const std::string &name) const { return getOutputIdx(name) != static_cast<std::size_t>(-1); }
  std::size_t getOutputIdx(const std::string &name, bool assert = false) const;
  const float *getOutput(const std::string &name) { return getOutput(getOutputIdx(name, true)); }
  virtual const float *getOutput(std::size_t idx) = 0;
  std::size_t getOutputSize(const std::string &name) const { return getOutputSize(getOutputIdx(name, true)); }
  std::size_t getOutputSize(std::size_t idx) const { return out_sizes_[idx]; }
  bool isOutputRecurrent(const std::string &name) const { return isOutputRecurrent(getOutputIdx(name, true)); }
  bool isOutputRecurrent(std::size_t idx) const { return out_recur_[idx]; }

  const auto &getRecurrentParams() const { return recur_params_; }
  const auto &getRecurrentParamIndices() const { return recur_param_indices_; }

 protected:
  virtual void synchronize() {}
  void postInit();

  std::string path_;
  yml::Node config_;
  std::size_t num_in_{}, num_out_{};
  std::vector<std::vector<int64_t>> in_shapes_, out_shapes_;
  std::vector<int64_t> in_sizes_, out_sizes_;
  std::vector<std::string> in_names_, out_names_;
  std::vector<bool> in_recur_, out_recur_;
  std::vector<std::pair<std::string, std::string>> recur_params_;
  std::vector<std::pair<std::size_t, std::size_t>> recur_param_indices_;
};
}  // namespace stepit

#define STEPIT_REGISTER_NNRTAPI(name, priority, factory) \
  static ::stepit::NnrtApi::Registration _nnrtapi_##name##_registration(#name, priority, factory)

#endif  // STEPIT_NNRT_H_

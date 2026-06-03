#ifndef STEPIT_NNRT_H_
#define STEPIT_NNRT_H_

#include <cstdint>
#include <string>
#include <vector>

#include <stepit/logging.h>
#include <stepit/registry.h>

namespace stepit {
enum class DataType { kFloat32, kInt32, kInt64, kBool };

std::size_t dataTypeSize(DataType dtype);
const char *dataTypeName(DataType dtype);

template <typename T>
struct DataTypeTrait;
template <>
struct DataTypeTrait<float> {
  static constexpr DataType value = DataType::kFloat32;
};
template <>
struct DataTypeTrait<int32_t> {
  static constexpr DataType value = DataType::kInt32;
};
template <>
struct DataTypeTrait<int64_t> {
  static constexpr DataType value = DataType::kInt64;
};
template <>
struct DataTypeTrait<bool> {
  static constexpr DataType value = DataType::kBool;
};

class Nnrt : public Interface<Nnrt, const std::string & /* path */, const yml::Node & /* config */> {
 public:
  Nnrt(const std::string &path, const yml::Node &config);
  Nnrt(const Nnrt &)            = delete;
  Nnrt &operator=(const Nnrt &) = delete;

  virtual void setInput(std::size_t idx, const void *data) = 0;
  virtual void runInference()                              = 0;
  virtual const void *getOutput(std::size_t idx)           = 0;
  virtual void clearState() {}
  void warmup(int iterations = 10);
  void printInfo() const;

  std::size_t getNumInputs() const { return num_in_; }
  const std::vector<std::string> &getInputNames() const { return in_names_; }
  const std::string &getInputName(std::size_t idx) const { return in_names_[idx]; }
  bool hasInput(const std::string &name) const { return getInputIndex(name) != static_cast<std::size_t>(-1); }
  std::size_t getInputIndex(const std::string &name, bool assert = false) const;
  std::size_t getInputSize(const std::string &name) const { return getInputSize(getInputIndex(name, true)); }
  std::size_t getInputSize(std::size_t idx) const { return in_sizes_[idx]; }
  std::size_t getInputBytes(const std::string &name) const { return getInputBytes(getInputIndex(name, true)); }
  std::size_t getInputBytes(std::size_t idx) const { return in_sizes_[idx] * dataTypeSize(in_dtypes_[idx]); }
  DataType getInputDtype(const std::string &name) const { return getInputDtype(getInputIndex(name, true)); }
  DataType getInputDtype(std::size_t idx) const { return in_dtypes_[idx]; }
  bool isInputRecurrent(const std::string &name) const { return isInputRecurrent(getInputIndex(name, true)); }
  bool isInputRecurrent(std::size_t idx) const { return in_recur_[idx]; }

  void setInput(const std::string &name, const void *data) { setInput(getInputIndex(name, true), data); }
  template <typename T>
  void setInput(std::size_t idx, const T *data) {
    STEPIT_ASSERT(in_dtypes_[idx] == DataTypeTrait<T>::value, "setInput<{}> called on input '{}' which has dtype {}.",
                  dataTypeName(DataTypeTrait<T>::value), in_names_[idx], dataTypeName(in_dtypes_[idx]));
    setInput(idx, static_cast<const void *>(data));
  }
  template <typename T>
  void setInput(const std::string &name, const T *data) {
    setInput<T>(getInputIndex(name, true), data);
  }

  std::size_t getNumOutputs() const { return num_out_; }
  const std::vector<std::string> &getOutputNames() const { return out_names_; }
  const std::string &getOutputName(std::size_t idx) const { return out_names_[idx]; }
  bool hasOutput(const std::string &name) const { return getOutputIndex(name) != static_cast<std::size_t>(-1); }
  std::size_t getOutputIndex(const std::string &name, bool assert = false) const;
  std::size_t getOutputSize(const std::string &name) const { return getOutputSize(getOutputIndex(name, true)); }
  std::size_t getOutputSize(std::size_t idx) const { return out_sizes_[idx]; }
  std::size_t getOutputBytes(const std::string &name) const { return getOutputBytes(getOutputIndex(name, true)); }
  std::size_t getOutputBytes(std::size_t idx) const { return out_sizes_[idx] * dataTypeSize(out_dtypes_[idx]); }
  DataType getOutputDtype(const std::string &name) const { return getOutputDtype(getOutputIndex(name, true)); }
  DataType getOutputDtype(std::size_t idx) const { return out_dtypes_[idx]; }
  bool isOutputRecurrent(const std::string &name) const { return isOutputRecurrent(getOutputIndex(name, true)); }
  bool isOutputRecurrent(std::size_t idx) const { return out_recur_[idx]; }

  const void *getOutput(const std::string &name) { return getOutput(getOutputIndex(name, true)); }
  template <typename T>
  const T *getOutput(std::size_t idx) {
    STEPIT_ASSERT(out_dtypes_[idx] == DataTypeTrait<T>::value,
                  "getOutput<{}> called on output '{}' which has dtype {}.", dataTypeName(DataTypeTrait<T>::value),
                  out_names_[idx], dataTypeName(out_dtypes_[idx]));
    return static_cast<const T *>(getOutput(idx));
  }
  template <typename T>
  const T *getOutput(const std::string &name) {
    return getOutput<T>(getOutputIndex(name, true));
  }
  const auto &getRecurrentParams() const { return recur_params_; }
  const auto &getRecurrentParamIndices() const { return recur_param_indices_; }

 protected:
  void addInput(std::string name, std::vector<int64_t> shape, int64_t size, DataType dtype) {
    in_names_.emplace_back(std::move(name));
    in_shapes_.emplace_back(std::move(shape));
    in_sizes_.push_back(size);
    in_dtypes_.push_back(dtype);
  }
  void addOutput(std::string name, std::vector<int64_t> shape, int64_t size, DataType dtype) {
    out_names_.emplace_back(std::move(name));
    out_shapes_.emplace_back(std::move(shape));
    out_sizes_.push_back(size);
    out_dtypes_.push_back(dtype);
  }
  void postInit();

  virtual void synchronize() {}

  std::string path_;
  yml::Node config_;
  std::size_t num_in_{}, num_out_{};
  std::vector<std::vector<int64_t>> in_shapes_, out_shapes_;
  std::vector<int64_t> in_sizes_, out_sizes_;
  std::vector<DataType> in_dtypes_, out_dtypes_;
  std::vector<std::string> in_names_, out_names_;
  std::vector<bool> in_recur_, out_recur_;
  std::vector<std::pair<std::string, std::string>> recur_params_;
  std::vector<std::pair<std::size_t, std::size_t>> recur_param_indices_;
};
}  // namespace stepit

#define STEPIT_REGISTER_NNRT(name, priority, factory) \
  static ::stepit::Nnrt::Registration _nnrt_##name##_registration(#name, priority, factory)

#endif  // STEPIT_NNRT_H_

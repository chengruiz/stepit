#ifndef STEPIT_FIELD_DATA_LOADER_H_
#define STEPIT_FIELD_DATA_LOADER_H_

#include <cstring>
#include <string>
#include <vector>

#include <stepit/registry.h>

namespace stepit {
namespace field {
struct NdArray {
  char *ptr{nullptr};
  std::vector<std::size_t> shape;
  std::size_t nbytes{};
  std::string dtype;
  std::size_t itemsize{};

  NdArray() = default;
  ~NdArray();
  NdArray(const NdArray &)            = delete;
  NdArray &operator=(const NdArray &) = delete;
  NdArray(NdArray &&other) noexcept;
  NdArray &operator=(NdArray &&other) noexcept;

  template <typename T>
  T *data() {
    return reinterpret_cast<T *>(ptr);
  }

  template <typename T>
  const T *data() const {
    return reinterpret_cast<const T *>(ptr);
  }

  std::size_t numel() const { return nbytes / itemsize; }

  template <typename T>
  T &at(std::size_t index) {
    return data<T>()[index];
  }

  template <typename T>
  const T &at(std::size_t index) const {
    return data<T>()[index];
  }

  template <typename T>
  Eigen::Map<Eigen::Array<T, -1, 1>> segment(std::size_t offset, std::size_t size) {
    return Eigen::Map<Eigen::Array<T, -1, 1>>(data<T>() + offset, size, 1);
  }

  template <typename T>
  Eigen::Map<const Eigen::Array<T, -1, 1>> segment(std::size_t offset, std::size_t size) const {
    return Eigen::Map<const Eigen::Array<T, -1, 1>>(data<T>() + offset, size, 1);
  }
};

class DataLoader : public Interface<DataLoader, std::string /* path */> {
 public:
  bool hasKey(const std::string &key) const;
  const std::vector<std::string> &keys() const { return keys_; }
  const NdArray &operator[](const std::string &key) const;

 protected:
  void clear();
  void insert(const std::string &key, NdArray array);

  std::vector<std::string> keys_;
  std::vector<NdArray> values_;
};

class CsvReader : public DataLoader {
 public:
  CsvReader(const std::string &path);

 private:
  NdArray readCsvFile(const std::string &path) const;

  bool skip_header_{true};
};
}  // namespace field
}  // namespace stepit

#define STEPIT_REGISTER_DATALOADER(name, priority, factory) \
  static ::stepit::field::DataLoader::Registration _data_loader_##name##_registration(#name, priority, factory)

#endif  // STEPIT_FIELD_DATA_LOADER_H_

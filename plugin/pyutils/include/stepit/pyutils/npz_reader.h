#include <cstring>
#include <string>
#include <vector>

#include <stepit/utils.h>

namespace stepit {
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
};

class NpzReader {
 public:
  NpzReader() = default;
  explicit NpzReader(const std::string &path);
  NpzReader(const NpzReader &)            = delete;
  NpzReader &operator=(const NpzReader &) = delete;
  NpzReader(NpzReader &&)                 = default;
  NpzReader &operator=(NpzReader &&)      = default;

  void readFile(const std::string &path);
  bool hasKey(const std::string &key) const;
  const std::vector<std::string> &keys() const { return keys_; }
  const NdArray &operator[](const std::string &key) const;

 private:
  std::vector<std::string> keys_;
  std::vector<NdArray> values_;
};
}  // namespace stepit

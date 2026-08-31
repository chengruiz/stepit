#ifndef STEPIT_DATA_TYPE_H_
#define STEPIT_DATA_TYPE_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

namespace stepit {
/** Scalar data types. */
enum class DataType { kUndefined, kFloat32, kInt32, kInt64, kBool };

/** Returns whether the data type is floating point. */
bool isFloatingPoint(DataType dtype);
/** Returns one scalar's byte size, or zero for `kUndefined` or an invalid value. */
std::size_t dataTypeSize(DataType dtype);
/** Returns a stable diagnostic name. */
const char *dataTypeName(DataType dtype);
/** Parses a concrete configuration data type. */
DataType parseDataType(const std::string &name);

/** Owning contiguous storage for one runtime scalar data type. */
class TypedBuffer {
 public:
  /** Creates an unallocated buffer with undefined data type. */
  TypedBuffer() = default;
  /** Allocates uninitialized storage for a concrete scalar data type. */
  TypedBuffer(DataType dtype, std::size_t size);

  TypedBuffer(const TypedBuffer &other);
  TypedBuffer &operator=(const TypedBuffer &other);
  TypedBuffer(TypedBuffer &&other) noexcept;
  TypedBuffer &operator=(TypedBuffer &&other) noexcept;
  ~TypedBuffer() = default;

  std::byte *data() { return static_cast<std::byte *>(data_.get()); }
  const std::byte *data() const { return static_cast<const std::byte *>(data_.get()); }
  std::size_t size() const { return size_; }
  std::size_t byteSize() const { return size_ * dataTypeSize(dtype_); }
  DataType dataType() const { return dtype_; }

  /** Replaces this buffer with uninitialized storage for a concrete scalar data type. */
  void allocate(DataType dtype, std::size_t size);

  void swap(TypedBuffer &other) noexcept {
    using std::swap;
    swap(dtype_, other.dtype_);
    swap(size_, other.size_);
    swap(data_, other.data_);
  }

 private:
  struct Deleter {
    DataType dtype{DataType::kUndefined};

    void operator()(void *data) const noexcept;
  };

  static void *allocateData(DataType dtype, std::size_t size);

  DataType dtype_{DataType::kUndefined};
  std::size_t size_{};
  std::unique_ptr<void, Deleter> data_{nullptr, Deleter{}};
};

/** Maps a supported C++ scalar type to its runtime data type. */
template <typename T>
struct DataTypeTrait;
template <>
struct DataTypeTrait<float> {
  static constexpr DataType value = DataType::kFloat32;
};
template <>
struct DataTypeTrait<std::int32_t> {
  static constexpr DataType value = DataType::kInt32;
};
template <>
struct DataTypeTrait<std::int64_t> {
  static constexpr DataType value = DataType::kInt64;
};
template <>
struct DataTypeTrait<bool> {
  static constexpr DataType value = DataType::kBool;
};
}  // namespace stepit

#endif  // STEPIT_DATA_TYPE_H_

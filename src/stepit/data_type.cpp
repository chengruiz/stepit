#include <stepit/data_type.h>

#include <cstring>
#include <stdexcept>

#include <stepit/utils.h>

namespace stepit {
bool isFloatingPoint(DataType dtype) {
  switch (dtype) {
    case DataType::kFloat32:
      return true;
    case DataType::kUndefined:
    case DataType::kInt32:
    case DataType::kInt64:
    case DataType::kBool:
      return false;
  }
  return false;
}

std::size_t dataTypeSize(DataType dtype) {
  switch (dtype) {
    case DataType::kUndefined:
      return 0;
    case DataType::kFloat32:
      return sizeof(float);
    case DataType::kInt32:
      return sizeof(std::int32_t);
    case DataType::kInt64:
      return sizeof(std::int64_t);
    case DataType::kBool:
      return sizeof(bool);
  }
  return 0;
}

const char *dataTypeName(DataType dtype) {
  switch (dtype) {
    case DataType::kUndefined:
      return "undefined";
    case DataType::kFloat32:
      return "float32";
    case DataType::kInt32:
      return "int32";
    case DataType::kInt64:
      return "int64";
    case DataType::kBool:
      return "bool";
  }
  return "unknown";
}

DataType parseDataType(const std::string &name) {
  if (name == "float32") return DataType::kFloat32;
  if (name == "int32") return DataType::kInt32;
  if (name == "int64") return DataType::kInt64;
  if (name == "bool") return DataType::kBool;
  throw std::invalid_argument("Unsupported data type '" + name + "'. Expected one of: float32, int32, int64, bool");
}

TypedBuffer::TypedBuffer(DataType dtype, std::size_t size) { allocate(dtype, size); }

TypedBuffer::TypedBuffer(const TypedBuffer &other) {
  if (other.dtype_ == DataType::kUndefined) return;
  allocate(other.dtype_, other.size_);
  if (size_ != 0) std::memcpy(data_.get(), other.data_.get(), byteSize());
}

TypedBuffer &TypedBuffer::operator=(const TypedBuffer &other) {
  if (this == &other) return *this;
  TypedBuffer copy(other);
  swap(copy);
  return *this;
}

TypedBuffer::TypedBuffer(TypedBuffer &&other) noexcept
    : dtype_(other.dtype_), size_(other.size_), data_(std::move(other.data_)) {
  other.dtype_ = DataType::kUndefined;
  other.size_  = 0;
}

TypedBuffer &TypedBuffer::operator=(TypedBuffer &&other) noexcept {
  if (this == &other) return *this;
  dtype_       = other.dtype_;
  size_        = other.size_;
  data_        = std::move(other.data_);
  other.dtype_ = DataType::kUndefined;
  other.size_  = 0;
  return *this;
}

void TypedBuffer::allocate(DataType dtype, std::size_t size) {
  STEPIT_ASSERT(dataTypeSize(dtype) != 0, "Cannot allocate typed storage with data type '{}'.", dataTypeName(dtype));
  std::unique_ptr<void, Deleter> data(nullptr, Deleter{dtype});
  if (size != 0) data.reset(allocateData(dtype, size));
  dtype_ = dtype;
  size_  = size;
  data_  = std::move(data);
}

void TypedBuffer::Deleter::operator()(void *data) const noexcept {
  switch (dtype) {
    case DataType::kUndefined:
      return;
    case DataType::kFloat32:
      delete[] static_cast<float *>(data);
      return;
    case DataType::kInt32:
      delete[] static_cast<std::int32_t *>(data);
      return;
    case DataType::kInt64:
      delete[] static_cast<std::int64_t *>(data);
      return;
    case DataType::kBool:
      delete[] static_cast<bool *>(data);
      return;
  }
}

void *TypedBuffer::allocateData(DataType dtype, std::size_t size) {
  switch (dtype) {
    case DataType::kUndefined:
      STEPIT_UNREACHABLE();
    case DataType::kFloat32:
      return new float[size];
    case DataType::kInt32:
      return new std::int32_t[size];
    case DataType::kInt64:
      return new std::int64_t[size];
    case DataType::kBool:
      return new bool[size];
  }
  STEPIT_UNREACHABLE();
}
}  // namespace stepit

#include <stepit/data_type.h>

#include <stdexcept>

namespace stepit {
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
  throw std::invalid_argument("Unsupported data type '" + name +
                              "'. Expected one of: float32, int32, int64, bool");
}
}  // namespace stepit

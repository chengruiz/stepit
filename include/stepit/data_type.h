#ifndef STEPIT_DATA_TYPE_H_
#define STEPIT_DATA_TYPE_H_

#include <cstddef>
#include <cstdint>
#include <string>

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

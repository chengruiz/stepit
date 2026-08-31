#include <stepit/field/field.h>

#include <cmath>
#include <cstring>
#include <limits>

namespace stepit {
namespace field {
namespace {
void validateFieldValue(FieldId field_id, const FieldValue &value, const FieldSpec &spec) {
  if (value.size() != spec.size or value.dataType() != spec.dtype) {
    STEPIT_THROW("Runtime value of field '{}' has data type '{}' and size {}, expected '{}' and {}.",
                 getFieldName(field_id), dataTypeName(value.dataType()), value.size(), dataTypeName(spec.dtype),
                 spec.size);
  }
}
}  // namespace

FieldValue::FieldValue(const FieldSpec &spec) {
  STEPIT_ASSERT(spec.isResolved(), "Cannot allocate field storage with size {} and data type {}.", spec.size,
                dataTypeName(spec.dtype));
  STEPIT_ASSERT(spec.size <= static_cast<std::size_t>(std::numeric_limits<Eigen::Index>::max()),
                "Field size {} exceeds the maximum Eigen index ({}).", spec.size,
                std::numeric_limits<Eigen::Index>::max());

  const auto size = static_cast<Eigen::Index>(spec.size);
  switch (spec.dtype) {
    case DataType::kUndefined:
      break;
    case DataType::kFloat32:
      storage_.emplace<FieldArray<float>>(size);
      break;
    case DataType::kInt32:
      storage_.emplace<FieldArray<std::int32_t>>(size);
      break;
    case DataType::kInt64:
      storage_.emplace<FieldArray<std::int64_t>>(size);
      break;
    case DataType::kBool:
      storage_.emplace<FieldArray<bool>>(size);
      break;
  }
}

std::byte *FieldValue::data() {
  return std::visit([](auto &value) { return reinterpret_cast<std::byte *>(value.data()); }, storage_);
}

const std::byte *FieldValue::data() const {
  return std::visit(
      [](const auto &value) { return reinterpret_cast<const std::byte *>(value.data()); }, storage_);
}

FieldSize FieldValue::size() const {
  return std::visit([](const auto &value) { return static_cast<std::size_t>(value.size()); }, storage_);
}

DataType FieldValue::dataType() const {
  return std::visit(
      [](const auto &value) {
        using Value  = typename std::decay<decltype(value)>::type;
        using Scalar = typename Value::Scalar;
        return DataTypeTrait<Scalar>::value;
      },
      storage_);
}

void FieldValue::copyFrom(const FieldValue &source, FieldSize source_offset, FieldSize target_offset,
                          FieldSize count) {
  STEPIT_ASSERT(source.dataType() == dataType(), "Cannot copy field data from type '{}' to type '{}'.",
                dataTypeName(source.dataType()), dataTypeName(dataType()));
  STEPIT_ASSERT(source_offset <= source.size() and count <= source.size() - source_offset,
                "Source range with offset {} and count {} exceeds field value size {}.", source_offset, count,
                source.size());
  copyFrom(static_cast<const void *>(source.data()), source_offset, target_offset, count);
}

void FieldValue::copyFrom(const void *source, FieldSize source_offset, FieldSize target_offset, FieldSize count) {
  STEPIT_ASSERT(target_offset <= size() and count <= size() - target_offset,
                "Target range with offset {} and count {} exceeds field value size {}.", target_offset, count, size());
  if (count == 0) return;
  STEPIT_ASSERT(source != nullptr, "Cannot copy field data from a null source.");

  const std::size_t element_size = dataTypeSize(dataType());
  const auto *source_data        = static_cast<const std::byte *>(source);
  std::memmove(data() + target_offset * element_size, source_data + source_offset * element_size,
               count * element_size);
}

FieldValue FieldValue::cast(DataType dtype) const {
  FieldValue result(FieldSpec{dtype, size()});
  std::visit(
      [](const auto &source, auto &target) {
        using Source = typename std::decay<decltype(source)>::type::Scalar;
        using Target = typename std::decay<decltype(target)>::type::Scalar;
        constexpr bool float_to_integer =
            std::is_same<Source, float>::value and
            (std::is_same<Target, std::int32_t>::value or std::is_same<Target, std::int64_t>::value);
        constexpr bool narrowing_integer =
            std::is_same<Source, std::int64_t>::value and std::is_same<Target, std::int32_t>::value;
        if constexpr (float_to_integer or narrowing_integer) {
          for (Eigen::Index index{}; index < source.size(); ++index) {
            const long double element = static_cast<long double>(source(index));
            bool valid = element >= static_cast<long double>(std::numeric_limits<Target>::lowest()) and
                         element <= static_cast<long double>(std::numeric_limits<Target>::max());
            if constexpr (float_to_integer) valid = valid and std::isfinite(source(index));
            STEPIT_ASSERT(valid, "Cannot cast element {} from {} value {} to {}.", index,
                          dataTypeName(DataTypeTrait<Source>::value), source(index),
                          dataTypeName(DataTypeTrait<Target>::value));
          }
        }
        target = source.template cast<Target>();
      },
      storage_, result.storage_);
  return result;
}

FieldId Node::registerRequirement(const std::string &field_name, DataType dtype, FieldSize field_size) {
  return registerRequirement(registerField(field_name, dtype, field_size));
}

FieldId Node::registerRequirement(FieldId field_id) {
  // If the field is not already registered as a provision, register it as a requirement.
  if (provisions_.find(field_id) == provisions_.end()) {
    requirements_.insert(field_id);
  }
  return field_id;
}

FieldId Node::registerProvision(const std::string &field_name, DataType dtype, FieldSize field_size) {
  return registerProvision(registerField(field_name, dtype, field_size));
}

FieldId Node::registerProvision(FieldId field_id) {
  // If the field is not already registered as a requirement, register it as a provision.
  if (requirements_.find(field_id) == requirements_.end()) {
    provisions_.insert(field_id);
  }
  return field_id;
}

ConflictingFieldSizeError::ConflictingFieldSizeError(FieldId field_id, FieldSize new_size)
    : std::runtime_error(fmt::format(
          "Attempting to register size of field '{}' to {}, which conflicts with the previously registered size ({}).",
          getFieldName(field_id), new_size, getFieldSize(field_id))) {}

ConflictingFieldDataTypeError::ConflictingFieldDataTypeError(FieldId field_id, DataType new_dtype)
    : std::runtime_error(fmt::format(
          "Attempting to register data type of field '{}' to {}, which conflicts with the previously registered type "
          "({}).",
          getFieldName(field_id), dataTypeName(new_dtype), dataTypeName(getFieldDataType(field_id)))) {}

UndefinedFieldSizeError::UndefinedFieldSizeError(FieldId field_id)
    : UndefinedFieldSpecError(fmt::format("Size of field '{}' is undefined.", getFieldName(field_id))) {}

UndefinedFieldDataTypeError::UndefinedFieldDataTypeError(FieldId field_id)
    : UndefinedFieldSpecError(fmt::format("Data type of field '{}' is undefined.", getFieldName(field_id))) {}

InvalidFieldIdError::InvalidFieldIdError(FieldId field_id)
    : std::runtime_error(fmt::format("Invalid field ID {}, which exceeds the number of registered fields ({}).",
                                     field_id, getNumFields())) {}

FieldId FieldManager::registerField(const std::string &name, DataType dtype, FieldSize size) {
  STEPIT_ASSERT(not name.empty(), "Field name should not be empty.");
  STEPIT_ASSERT(dtype == DataType::kUndefined or dataTypeSize(dtype) != 0, "Field '{}' has an invalid data type.",
                name);

  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto it = name_to_id_.find(name);
  if (it == name_to_id_.end()) {  // If not registered
    const FieldId id   = next_id_++;
    name_to_id_[name]  = id;
    id_to_name_.push_back(name);
    id_to_spec_.push_back(FieldSpec{dtype, size});
    return id;
  }

  const FieldId id = it->second;
  setFieldSpec(id, FieldSpec{dtype, size});
  return id;
}

FieldId FieldManager::getFieldId(const std::string &name) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto it = name_to_id_.find(name);
  if (it == name_to_id_.end()) throw UnregisteredFieldError(name);
  return it->second;
}

std::string FieldManager::getFieldName(FieldId id) const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (id >= next_id_) throw InvalidFieldIdError(id);
  return id_to_name_[id];
}

FieldId FieldManager::getNumFields() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return next_id_;
}

FieldSpec FieldManager::getFieldSpec(FieldId id) const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (id >= next_id_) throw InvalidFieldIdError(id);
  const FieldSpec spec = id_to_spec_[id];
  if (spec.size == 0) throw UndefinedFieldSizeError(id);
  if (spec.dtype == DataType::kUndefined) throw UndefinedFieldDataTypeError(id);
  return spec;
}

FieldSize FieldManager::getFieldSize(FieldId id) const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (id >= next_id_) throw InvalidFieldIdError(id);
  const FieldSize size = id_to_spec_[id].size;
  if (size == 0) throw UndefinedFieldSizeError(id);
  return size;
}

DataType FieldManager::getFieldDataType(FieldId id) const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (id >= next_id_) throw InvalidFieldIdError(id);
  const DataType dtype = id_to_spec_[id].dtype;
  if (dtype == DataType::kUndefined) throw UndefinedFieldDataTypeError(id);
  return dtype;
}

void FieldManager::setFieldSpec(FieldId id, const FieldSpec &spec) {
  STEPIT_ASSERT(spec.dtype == DataType::kUndefined or dataTypeSize(spec.dtype) != 0,
                "Cannot set an invalid field data type.");

  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (id >= next_id_) throw InvalidFieldIdError(id);
  auto &registered = id_to_spec_[id];

  if (spec.size != 0 and registered.size != 0 and registered.size != spec.size) {
    throw ConflictingFieldSizeError(id, spec.size);
  }
  if (spec.dtype != DataType::kUndefined and registered.dtype != DataType::kUndefined and
      registered.dtype != spec.dtype) {
    throw ConflictingFieldDataTypeError(id, spec.dtype);
  }

  if (spec.size != 0 and registered.size == 0) {
    STEPIT_DBUGNT("Size of field '{}' set to {}.", getFieldName(id), spec.size);
    registered.size = spec.size;
  }

  if (spec.dtype != DataType::kUndefined and registered.dtype == DataType::kUndefined) {
    STEPIT_DBUGNT("Data type of field '{}' set to {}.", getFieldName(id), dataTypeName(spec.dtype));
    registered.dtype = spec.dtype;
  }
}

FieldManager &FieldManager::instance() {
  static FieldManager instance;
  return instance;
}

FieldValue parseFieldValue(const yml::Node &node, const FieldSpec &spec, bool allow_missing) {
  STEPIT_ASSERT(spec.isResolved(), "Cannot parse a field value with data type '{}' and size {}.",
                dataTypeName(spec.dtype), spec.size);
  FieldValue result(spec);
  switch (spec.dtype) {
    case DataType::kUndefined:
      STEPIT_UNREACHABLE();
    case DataType::kFloat32: {
      auto &value = result.get<float>();
      if (allow_missing and not node.hasValue()) value.setZero();
      node.to(value, allow_missing);
      break;
    }
    case DataType::kInt32: {
      auto &value = result.get<std::int32_t>();
      if (allow_missing and not node.hasValue()) value.setZero();
      node.to(value, allow_missing);
      break;
    }
    case DataType::kInt64: {
      auto &value = result.get<std::int64_t>();
      if (allow_missing and not node.hasValue()) value.setZero();
      node.to(value, allow_missing);
      break;
    }
    case DataType::kBool: {
      auto &value = result.get<bool>();
      if (allow_missing and not node.hasValue()) value.setZero();
      node.to(value, allow_missing);
      break;
    }
  }
  return result;
}

const FieldValue &readFieldValue(const FieldMap &context, FieldId field_id) {
  const auto it = context.find(field_id);
  if (it == context.end()) {
    STEPIT_THROW("Field '{}' is not available in the runtime context.", getFieldName(field_id));
  }
  validateFieldValue(field_id, it->second, getFieldSpec(field_id));
  return it->second;
}

FieldValue &ensureFieldValue(FieldMap &context, FieldId field_id) {
  const FieldSpec spec = getFieldSpec(field_id);
  auto result          = context.try_emplace(field_id, spec);
  validateFieldValue(field_id, result.first->second, spec);
  return result.first->second;
}

void parseFieldIds(const yml::Node &node, FieldIdVec &result) {
  node.assertSequence();
  for (const auto &item : node) result.push_back(getFieldId(item.as<std::string>()));
}
}  // namespace field
}  // namespace stepit

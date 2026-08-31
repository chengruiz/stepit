#ifndef STEPIT_FIELD_H_
#define STEPIT_FIELD_H_

#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include <stepit/data_type.h>
#include <stepit/registry.h>
#include <stepit/utils.h>

namespace stepit {
namespace field {
/** Unique identifier of a registered field. */
using FieldId = std::size_t;
/** Declared scalar length of one field segment. */
using FieldSize = std::size_t;
/** Ordered field ID list used for concat/split layouts. */
using FieldIdVec = std::vector<FieldId>;

/** One-dimensional Eigen array used for field values and views. */
template <typename T>
using FieldArray = Eigen::Array<T, Eigen::Dynamic, 1>;
/** Mutable non-owning view of a field array. */
template <typename T>
using FieldView = Eigen::Map<FieldArray<T>>;
/** Read-only non-owning view of a field array. */
template <typename T>
using ConstFieldView = Eigen::Map<const FieldArray<T>>;

/** Field metadata; zero size or `kUndefined` denotes an unresolved component. */
struct FieldSpec {
  /** Returns whether both size and data type are concrete and valid. */
  bool isResolved() const { return size != 0 and dataTypeSize(dtype) != 0; }
  /** Returns the number of bytes required by a fully resolved field. */
  std::size_t byteSize() const {
    STEPIT_ASSERT(isResolved(), "Cannot get the byte size of a field with data type '{}' and size {}.",
                  dataTypeName(dtype), size);
    return size * dataTypeSize(dtype);
  }

  DataType dtype{DataType::kUndefined};
  FieldSize size{};
};

/**
 * Owning runtime value of a registered field.
 *
 * A value owns a typed buffer. Its default state has no data type or storage.
 */
class FieldValue {
 public:
  FieldValue() = default;

  /** Allocates storage described by a fully resolved field specification. */
  explicit FieldValue(const FieldSpec &spec);

  /** Copies an Eigen row or column vector into owning field storage. */
  template <typename Derived>
  FieldValue(const Eigen::DenseBase<Derived> &value) {
    assign(value);
  }

  /** Replaces this value with a copy of an Eigen row or column vector. */
  template <typename Derived>
  FieldValue &operator=(const Eigen::DenseBase<Derived> &value) {
    assign(value);
    return *this;
  }

  /** Returns a mutable Eigen view after validating the scalar data type. */
  template <typename T>
  FieldView<T> view() {
    static_assert(isSupportedScalar<T>(), "Unsupported field scalar type.");
    STEPIT_ASSERT(dataType() == DataTypeTrait<T>::value, "Cannot view field value with data type '{}' as '{}'.",
                  dataTypeName(dataType()), dataTypeName(DataTypeTrait<T>::value));
    return FieldView<T>(reinterpret_cast<T *>(data()), static_cast<Eigen::Index>(size()));
  }

  /** Returns a read-only Eigen view after validating the scalar data type. */
  template <typename T>
  ConstFieldView<T> view() const {
    static_assert(isSupportedScalar<T>(), "Unsupported field scalar type.");
    STEPIT_ASSERT(dataType() == DataTypeTrait<T>::value, "Cannot view field value with data type '{}' as '{}'.",
                  dataTypeName(dataType()), dataTypeName(DataTypeTrait<T>::value));
    return ConstFieldView<T>(reinterpret_cast<const T *>(data()), static_cast<Eigen::Index>(size()));
  }

  /** Returns mutable byte access to the contiguous scalar storage. */
  std::byte *data() { return buffer_.data(); }
  /** Returns read-only byte access to the contiguous scalar storage. */
  const std::byte *data() const { return buffer_.data(); }
  /** Returns the scalar length of this value. */
  FieldSize size() const { return buffer_.size(); }
  /** Returns the scalar data type of this value. */
  DataType dataType() const { return buffer_.dataType(); }
  /** Copies a scalar range from another value of the same data type. */
  void copyFrom(const FieldValue &source, FieldSize source_offset, FieldSize target_offset, FieldSize count);
  /** Copies a scalar range from a raw buffer containing this value's data type. */
  void copyFrom(const void *source, FieldSize source_offset, FieldSize target_offset, FieldSize count);
  /** Returns an element-wise scalar conversion with the same length. */
  FieldValue cast(DataType dtype) const;

 private:
  template <typename T>
  static constexpr bool isSupportedScalar() {
    return std::is_same<T, float>::value or std::is_same<T, std::int32_t>::value or
           std::is_same<T, std::int64_t>::value or std::is_same<T, bool>::value;
  }

  template <typename Derived>
  void assign(const Eigen::DenseBase<Derived> &value) {
    using Scalar = typename std::remove_cv<typename Derived::Scalar>::type;
    static_assert(isSupportedScalar<Scalar>(), "Unsupported field scalar type.");
    STEPIT_ASSERT(value.rows() == 1 or value.cols() == 1, "Field value must be a vector, got shape [{} x {}].",
                  value.rows(), value.cols());
    TypedBuffer new_buffer(DataTypeTrait<Scalar>::value, static_cast<std::size_t>(value.size()));
    FieldView<Scalar> result(reinterpret_cast<Scalar *>(new_buffer.data()), value.size());
    Eigen::Index index{};
    for (Eigen::Index col{}; col < value.cols(); ++col) {
      for (Eigen::Index row{}; row < value.rows(); ++row) result(index++) = value.derived().coeff(row, col);
    }
    buffer_ = std::move(new_buffer);
  }

  TypedBuffer buffer_;
};

/** Runtime map from field ID to the value available in the current context. */
using FieldMap = std::map<FieldId, FieldValue>;

/** Sentinel value used to represent an invalid field ID. */
constexpr FieldId kInvalidFieldId = static_cast<FieldId>(-1);

/**
 * Base mixin for components that consume and/or produce fields.
 *
 * `requirements_` tracks fields the node expects in context.
 * `provisions_` tracks fields the node writes into context.
 */
class Node {
 public:
  /** Returns all fields required by this node. */
  const std::set<FieldId> &requirements() const { return requirements_; }
  /** Returns all fields provided by this node. */
  const std::set<FieldId> &provisions() const { return provisions_; }

 protected:
  /** Registers a required field by name, optionally defining its specification. */
  FieldId registerRequirement(const std::string &field_name, DataType dtype = DataType::kUndefined,
                              FieldSize field_size = 0);
  /** Registers a required field by ID. */
  FieldId registerRequirement(FieldId field_id);
  /** Registers a provided field by name, optionally defining its specification. */
  FieldId registerProvision(const std::string &field_name, DataType dtype = DataType::kUndefined,
                            FieldSize field_size = 0);
  /** Registers a provided field by ID. */
  FieldId registerProvision(FieldId field_id);

 private:
  std::set<FieldId> requirements_, provisions_;
};

/** Thrown when attempting to set a field size that conflicts with an existing size. */
struct ConflictingFieldSizeError : std::runtime_error {
  explicit ConflictingFieldSizeError(FieldId field_id, FieldSize new_size);
};

/** Thrown when attempting to set a field data type that conflicts with an existing type. */
struct ConflictingFieldDataTypeError : std::runtime_error {
  ConflictingFieldDataTypeError(FieldId field_id, DataType new_dtype);
};

/** Common base for errors caused by unresolved field metadata. */
struct UndefinedFieldSpecError : std::runtime_error {
  explicit UndefinedFieldSpecError(const std::string &message) : std::runtime_error(message) {}
};

/** Thrown when a field size is requested before being defined. */
struct UndefinedFieldSizeError : UndefinedFieldSpecError {
  explicit UndefinedFieldSizeError(FieldId field_id);
};

/** Thrown when a field data type is requested before being defined. */
struct UndefinedFieldDataTypeError : UndefinedFieldSpecError {
  explicit UndefinedFieldDataTypeError(FieldId field_id);
};

/** Thrown when a field ID is outside the current registry range. */
struct InvalidFieldIdError : std::runtime_error {
  explicit InvalidFieldIdError(FieldId field_id);
};

/** Thrown when requesting the ID of an unregistered field name. */
struct UnregisteredFieldError : std::runtime_error {
  explicit UnregisteredFieldError(const std::string &field_name)
      : std::runtime_error(fmt::format("Field '{}' is not registered.", field_name)) {}
};

/**
 * Singleton registry that owns field IDs, names, and declared specifications.
 *
 * Registration is append-only during process lifetime. Re-registering a name
 * merges newly defined size and data-type components and rejects conflicts.
 */
class FieldManager {
 public:
  FieldManager(const FieldManager &)            = delete;
  FieldManager &operator=(const FieldManager &) = delete;

  /** Returns the process-wide field manager instance. */
  static FieldManager &instance();

  /** Registers a field name and optional specification. Returns the assigned field ID. */
  FieldId registerField(const std::string &name, DataType dtype = DataType::kUndefined, FieldSize size = 0);
  /** Returns the field ID for a registered name. */
  FieldId getFieldId(const std::string &name);
  /** Returns the registered field name for a valid ID. */
  std::string getFieldName(FieldId id) const;
  /** Returns total number of registered fields. */
  FieldId getNumFields() const;
  /** Returns a fully resolved field specification. */
  FieldSpec getFieldSpec(FieldId id) const;
  /** Returns field size for a valid ID; throws if undefined. */
  FieldSize getFieldSize(FieldId id) const;
  /** Returns field data type for a valid ID; throws if undefined. */
  DataType getFieldDataType(FieldId id) const;
  /** Merges defined components into an existing field specification. */
  void setFieldSpec(FieldId id, const FieldSpec &spec);

 private:
  FieldManager() = default;

  mutable std::recursive_mutex mutex_;
  std::map<std::string, FieldId> name_to_id_;
  std::vector<std::string> id_to_name_;
  std::vector<FieldSpec> id_to_spec_;
  FieldId next_id_{};
};

// Convenience accessors
inline FieldManager &fieldManager() { return FieldManager::instance(); }
inline FieldId registerField(const std::string &name, DataType dtype = DataType::kUndefined, FieldSize size = 0) {
  return fieldManager().registerField(name, dtype, size);
}
inline FieldId getFieldId(const std::string &name) { return fieldManager().getFieldId(name); }
inline std::string getFieldName(FieldId id) { return fieldManager().getFieldName(id); }
inline FieldId getNumFields() { return fieldManager().getNumFields(); }
inline FieldSpec getFieldSpec(FieldId id) { return fieldManager().getFieldSpec(id); }
inline FieldSize getFieldSize(FieldId id) { return fieldManager().getFieldSize(id); }
inline DataType getFieldDataType(FieldId id) { return fieldManager().getFieldDataType(id); }
inline void setFieldSpec(FieldId id, const FieldSpec &spec) { fieldManager().setFieldSpec(id, spec); }

/** Decodes a YAML scalar or sequence; missing values become zero when allowed. */
FieldValue parseFieldValue(const yml::Node &node, const FieldSpec &spec, bool allow_missing = false);
/** Returns a field present in the runtime context after validating its registered specification. */
const FieldValue &readFieldValue(const FieldMap &context, FieldId field_id);
/** Ensures that a correctly specified runtime value exists and returns it. */
FieldValue &ensureFieldValue(FieldMap &context, FieldId field_id);

/** Parses a YAML sequence of field names into a list of field IDs. */
void parseFieldIds(const yml::Node &node, FieldIdVec &result);
}  // namespace field
}  // namespace stepit

#endif  // STEPIT_FIELD_H_

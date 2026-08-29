#ifndef STEPIT_FIELD_H_
#define STEPIT_FIELD_H_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <mutex>
#include <set>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <stepit/data_type.h>
#include <stepit/registry.h>
#include <stepit/utils.h>

namespace stepit {
namespace field {
/** Unique identifier of a registered field. */
using FieldId = std::size_t;
/** Declared scalar length of one field segment. */
using FieldSize = std::uint32_t;
/** Ordered field ID list used for concat/split layouts. */
using FieldIdVec = std::vector<FieldId>;

/** One-dimensional owning array used as field storage. */
template <typename T>
using FieldArray = Eigen::Array<T, Eigen::Dynamic, 1>;

/** Owning runtime value of a field. */
class FieldValue {
 public:
  FieldValue() = default;

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

  /** Returns the stored array, throwing `std::bad_variant_access` for a type mismatch. */
  template <typename T>
  FieldArray<T> &get() {
    static_assert(isSupportedScalar<T>(), "Unsupported field scalar type.");
    return std::get<FieldArray<T>>(storage_);
  }

  /** Returns the stored array, throwing `std::bad_variant_access` for a type mismatch. */
  template <typename T>
  const FieldArray<T> &get() const {
    static_assert(isSupportedScalar<T>(), "Unsupported field scalar type.");
    return std::get<FieldArray<T>>(storage_);
  }

  /** Returns mutable access to the contiguous scalar storage. */
  void *data();
  /** Returns read-only access to the contiguous scalar storage. */
  const void *data() const;
  /** Returns the number of stored scalars. */
  FieldSize size() const;
  /** Returns the stored scalar data type. */
  DataType dataType() const;

 private:
  using Storage =
      std::variant<FieldArray<float>, FieldArray<std::int32_t>, FieldArray<std::int64_t>, FieldArray<bool>>;

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
    STEPIT_ASSERT(static_cast<std::uint64_t>(value.size()) <= std::numeric_limits<FieldSize>::max(),
                  "Field value size {} exceeds the maximum supported size ({}).", value.size(),
                  std::numeric_limits<FieldSize>::max());

    FieldArray<Scalar> result(value.size());
    Eigen::Index index{};
    for (Eigen::Index col{}; col < value.cols(); ++col) {
      for (Eigen::Index row{}; row < value.rows(); ++row) result(index++) = value.derived().coeff(row, col);
    }
    storage_ = std::move(result);
  }

  Storage storage_;
};

/** Runtime map from field ID to its current value. */
using FieldMap = std::map<FieldId, FieldValue>;

/** Field metadata; zero size or `kUndefined` denotes an unresolved component. */
struct FieldSpec {
  DataType dtype{DataType::kUndefined};
  FieldSize size{};
};

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

/** Returns a field present in the runtime context after validating its registered specification. */
const FieldValue &readFieldValue(const FieldMap &context, FieldId field_id);

/** Parses a YAML sequence of field names into a list of field IDs. */
void parseFieldIds(const yml::Node &node, FieldIdVec &result);
/** Copies a vector into a result buffer at offset, then advances the offset. */
void stackField(cArrXf vec, uint32_t &index, rArrXf result);
/** Concatenates selected fields from context into a pre-sized destination vector. */
void concatFields(const FieldMap &context, const FieldIdVec &field_ids, rArrXf result);
/** Splits a source vector into field slices according to registered field sizes. */
void splitFields(cArrXf source, const FieldIdVec &field_ids, FieldMap &result);
}  // namespace field
}  // namespace stepit

#endif  // STEPIT_FIELD_H_

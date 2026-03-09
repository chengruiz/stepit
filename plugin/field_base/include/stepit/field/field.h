#ifndef STEPIT_FIELD_H_
#define STEPIT_FIELD_H_

#include <cstdint>
#include <map>
#include <mutex>
#include <set>
#include <stdexcept>
#include <vector>

#include <stepit/registry.h>
#include <stepit/utils.h>

namespace stepit {
namespace field {
/** Unique identifier of a registered field. */
using FieldId = std::size_t;
/** Runtime map from field ID to field value vector. */
using FieldMap = std::map<FieldId, ArrXf>;
/** Declared scalar length of one field segment. */
using FieldSize = std::uint32_t;
/** Ordered field ID list used for concat/split layouts. */
using FieldIdVec = std::vector<FieldId>;

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
  /** Registers a required field by name, optionally defining its size. */
  FieldId registerRequirement(const std::string &field_name, FieldSize field_size = 0);
  /** Registers a required field by ID. */
  FieldId registerRequirement(FieldId field_id);
  /** Registers a provided field by name and size. */
  FieldId registerProvision(const std::string &field_name, FieldSize field_size);
  /** Registers a provided field by ID. */
  FieldId registerProvision(FieldId field_id);

 private:
  std::set<FieldId> requirements_, provisions_;
};

/** Thrown when attempting to set a field size that conflicts with an existing size. */
struct ConflictingFieldSizeError : std::runtime_error {
  explicit ConflictingFieldSizeError(FieldId field_id, FieldSize new_size);
};

/** Thrown when a field ID is outside the current registry range. */
struct InvalidFieldIdError : std::runtime_error {
  explicit InvalidFieldIdError(FieldId field_id);
};

/** Thrown when a field size is requested before being defined. */
struct UndefinedFieldSizeError : std::runtime_error {
  explicit UndefinedFieldSizeError(FieldId field_id);
};

/** Thrown when requesting the ID of an unregistered field name. */
struct UnregisteredFieldError : std::runtime_error {
  explicit UnregisteredFieldError(const std::string &field_name)
      : std::runtime_error(fmt::format("Field '{}' is not registered.", field_name)) {}
};

/**
 * Singleton registry that owns field IDs, names, and declared sizes.
 *
 * Registration is append-only during process lifetime.
 */
class FieldManager {
 public:
  FieldManager(const FieldManager &)            = delete;
  FieldManager &operator=(const FieldManager &) = delete;

  /** Returns the process-wide field manager instance. */
  static FieldManager &instance();

  /** Registers a field name and optional size. Returns the assigned field ID. */
  FieldId registerField(const std::string &name, FieldSize size);
  /** Returns the field ID for a registered name. */
  FieldId getFieldId(const std::string &name);
  /** Returns the registered field name for a valid ID. */
  std::string getFieldName(FieldId id) const;
  /** Returns total number of registered fields. */
  FieldId getNumFields() const;
  /** Returns field size for a valid ID; throws if undefined. */
  FieldSize getFieldSize(FieldId id) const;
  /** Sets size for an existing field ID or validates the existing size. */
  void setFieldSize(FieldId id, FieldSize size);

 private:
  FieldManager() = default;

  mutable std::recursive_mutex mutex_;
  std::map<std::string, FieldId> name_to_id_;
  std::vector<std::string> id_to_name_;
  std::vector<FieldSize> id_to_size_;
  FieldId next_id_{};
};

// Convenience accessors
inline FieldManager &fieldManager() { return FieldManager::instance(); }
inline FieldId registerField(const std::string &name, FieldSize size) {
  return fieldManager().registerField(name, size);
}
inline FieldId getFieldId(const std::string &name) { return fieldManager().getFieldId(name); }
inline std::string getFieldName(FieldId id) { return fieldManager().getFieldName(id); }
inline FieldId getNumFields() { return fieldManager().getNumFields(); }
inline FieldSize getFieldSize(FieldId id) { return fieldManager().getFieldSize(id); }
inline void setFieldSize(FieldId id, FieldSize size) { fieldManager().setFieldSize(id, size); }

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

#ifndef STEPIT_FIELD_H_
#define STEPIT_FIELD_H_

#include <cstdint>
#include <map>
#include <mutex>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

#include <stepit/registry.h>
#include <stepit/utils.h>

namespace stepit {
namespace field {
using FieldId    = std::size_t;
using FieldMap   = std::map<FieldId, ArrXf>;
using FieldSize  = std::uint32_t;
using FieldIdVec = std::vector<FieldId>;

constexpr FieldId kInvalidFieldId = static_cast<FieldId>(-1);

class Node {
 public:
  const std::set<FieldId> &requirements() const { return requirements_; }
  const std::set<FieldId> &provisions() const { return provisions_; }

 protected:
  FieldId registerRequirement(const std::string &field_name, FieldSize field_size = 0);
  FieldId registerRequirement(FieldId field_id);
  FieldId registerProvision(const std::string &field_name, FieldSize field_size);
  FieldId registerProvision(FieldId field_id);

  std::set<FieldId> requirements_, provisions_;
};

struct ConflictingFieldSizeError : std::runtime_error {
  explicit ConflictingFieldSizeError(FieldId field_id, FieldSize new_size);
};

struct InvalidFieldIdError : std::runtime_error {
  explicit InvalidFieldIdError(FieldId field_id);
};

struct UndefinedFieldSizeError : std::runtime_error {
  explicit UndefinedFieldSizeError(FieldId field_id);
};

struct UnregisteredFieldError : std::runtime_error {
  explicit UnregisteredFieldError(const std::string &field_name)
      : std::runtime_error(fmt::format("Field '{}' is not registered.", field_name)) {}
};

// Singleton registry to manage fields
class FieldManager {
 public:
  FieldManager(const FieldManager &)            = delete;
  FieldManager &operator=(const FieldManager &) = delete;
  static FieldManager &instance();

  FieldId registerField(const std::string &name, FieldSize size);
  FieldId getFieldId(const std::string &name);
  std::string getFieldName(FieldId id) const;
  FieldId getNumFields() const;
  FieldSize getFieldSize(FieldId id) const;
  void setFieldSize(FieldId id, FieldSize size);

 private:
  FieldManager() = default;

  mutable std::recursive_mutex mutex_;
  std::map<std::string, FieldId> name_to_id_;
  std::vector<std::string> id_to_name_;
  std::vector<FieldSize> id_to_size_;
  FieldId next_id_{};
};

// Helper accessors
inline FieldManager &fieldManager() { return FieldManager::instance(); }
inline FieldId registerField(const std::string &name, FieldSize size) {
  return fieldManager().registerField(name, size);
}
inline FieldId getFieldId(const std::string &name) { return fieldManager().getFieldId(name); }
inline std::string getFieldName(FieldId id) { return fieldManager().getFieldName(id); }
inline FieldId getNumFields() { return fieldManager().getNumFields(); }
inline FieldSize getFieldSize(FieldId id) { return fieldManager().getFieldSize(id); }
inline void setFieldSize(FieldId id, FieldSize size) { fieldManager().setFieldSize(id, size); }

void parseFieldIds(const YAML::Node &node, FieldIdVec &result);
void stackField(cArrXf vec, uint32_t &index, rArrXf result);
void concatFields(const FieldMap &context, const FieldIdVec &field_ids, rArrXf result);
void splitFields(cArrXf source, const FieldIdVec &field_ids, FieldMap &result);
}  // namespace field
}  // namespace stepit

#endif  // STEPIT_FIELD_H_

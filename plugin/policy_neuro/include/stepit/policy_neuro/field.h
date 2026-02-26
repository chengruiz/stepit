#ifndef STEPIT_NEURO_POLICY_FIELD_H_
#define STEPIT_NEURO_POLICY_FIELD_H_

#include <map>
#include <mutex>
#include <utility>
#include <vector>

#include <stepit/control_input.h>
#include <stepit/policy.h>
#include <stepit/registry.h>
#include <stepit/utils.h>

namespace stepit {
namespace neuro_policy {
using FieldId    = std::size_t;
using FieldMap   = std::map<FieldId, ArrXf>;
using FieldSize  = std::uint32_t;
using FieldIdVec = std::vector<FieldId>;

constexpr FieldId kInvalidFieldId = static_cast<FieldId>(-1);

struct NeuroPolicySpec : PolicySpec {
  using PolicySpec::PolicySpec;
  /* The default action to take */
  ArrXf default_action;
};

class Module : public Interface<Module, const NeuroPolicySpec & /* policy_spec */, const std::string & /* name */> {
 public:
  virtual void initFieldProperties() {}
  virtual bool reset() { return true; }
  virtual bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) = 0;
  virtual void postUpdate(const FieldMap &field_map) {}
  virtual void exit() {}

  const std::string &name() const { return name_; }
  const std::set<FieldId> &requirements() const { return requirements_; }
  const std::set<FieldId> &provisions() const { return provisions_; }

 protected:
  Module(const NeuroPolicySpec &policy_spec, std::string name, bool allow_config_missing = false);
  FieldId registerRequirement(const std::string &field_name, FieldSize field_size = 0);
  FieldId registerRequirement(FieldId field_id);
  FieldId registerProvision(const std::string &field_name, FieldSize field_size);
  FieldId registerProvision(FieldId field_id);

  const std::string name_, config_filename_;
  YAML::Node config_;
  std::set<FieldId> requirements_, provisions_;
};

// Singleton registry to manage observations
class FieldManager {
 public:
  FieldManager(const FieldManager &)            = delete;
  FieldManager &operator=(const FieldManager &) = delete;
  static FieldManager &instance();

  using SourceRegistry = Registry<Module, const NeuroPolicySpec &, const std::string &>;
  auto registerSource(const std::string &field_name, int priority, SourceRegistry::Factory factory)
      -> SourceRegistry::Registration;
  auto makeSource(const std::string &field_name, const NeuroPolicySpec &policy_spec, const std::string &name)
      -> Module::Ptr;

  FieldId registerField(const std::string &name, FieldSize size);
  FieldId getFieldId(const std::string &name);
  const std::string &getFieldName(FieldId id) const;
  FieldSize getFieldSize(FieldId id) const;
  void setFieldSize(FieldId id, FieldSize size);

 private:
  FieldManager() = default;
  mutable std::recursive_mutex mutex_;
  SourceRegistry source_registry_;
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
inline const std::string &getFieldName(FieldId id) { return fieldManager().getFieldName(id); }
inline FieldSize getFieldSize(FieldId id) { return fieldManager().getFieldSize(id); }
inline void setFieldSize(FieldId id, FieldSize size) { fieldManager().setFieldSize(id, size); }
inline auto makeFieldSource(const std::string &field_name, const NeuroPolicySpec &policy_spec, const std::string &name)
    -> Module::Ptr {
  return fieldManager().makeSource(field_name, policy_spec, name);
}

void parseFieldIds(const YAML::Node &node, FieldIdVec &result);
void stackField(cArrXf vec, uint32_t &index, rArrXf result);
void concatFields(const FieldMap &context, const FieldIdVec &field_ids, rArrXf result);
void splitFields(cArrXf source, const FieldIdVec &field_ids, FieldMap &result);
}  // namespace neuro_policy
}  // namespace stepit

#define STEPIT_REGISTER_MODULE(name, priority, factory) \
  static ::stepit::neuro_policy::Module::Registration _field_source_##name##_registration(#name, priority, factory)
#define STEPIT_REGISTER_FIELD_SOURCE(field_name, priority, factory)                              \
  static auto _field_##field_name##_source_registration = ::stepit::neuro_policy::fieldManager() \
                                                              .registerSource(#field_name, priority, factory)

#endif  // STEPIT_NEURO_POLICY_FIELD_H_

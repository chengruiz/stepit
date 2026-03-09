#ifndef STEPIT_NEURO_POLICY_FIELD_H_
#define STEPIT_NEURO_POLICY_FIELD_H_

#include <string>

#include <stepit/control_input.h>
#include <stepit/policy.h>
#include <stepit/field/field.h>

namespace stepit {
namespace neuro_policy {
using namespace ::stepit::field;

struct NeuroPolicySpec : PolicySpec {
  using PolicySpec::PolicySpec;
  /* The default action to take */
  ArrXf default_action;
};

struct ModuleSpec {
  ModuleSpec() = default;
  ModuleSpec(const std::string &name, const yml::Node &config = yml::Node()) : name(name), config(config) {}
  ModuleSpec(const ModuleSpec &other, const std::string &default_name)
      : name(nonEmptyOr(other.name, default_name)), config(other.config) {}

  std::string name;
  yml::Node config;
};

class Module : public Node,
               public Interface<Module, const NeuroPolicySpec & /* policy_spec */, const ModuleSpec & /* name */> {
 public:
  virtual void init() {}
  virtual bool reset() { return true; }
  virtual bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) = 0;
  virtual void commit(const FieldMap &context) {}
  virtual void exit() {}

  const std::string &name() const { return name_; }

 protected:
  Module(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec);

  const std::string name_, config_filename_;
  yml::Node config_;
};

class FieldSourceRegistry : public Registry<Module, const NeuroPolicySpec &, const std::string &> {
 public:
  static FieldSourceRegistry &instance();
};

inline auto makeFieldSource(const std::string &field_name, const NeuroPolicySpec &policy_spec, const std::string &name)
    -> Module::Ptr {
  return FieldSourceRegistry::instance().make(field_name, policy_spec, name);
}
}  // namespace neuro_policy
}  // namespace stepit

#define STEPIT_REGISTER_MODULE(name, priority, factory) \
  static ::stepit::neuro_policy::Module::Registration _field_source_##name##_registration(#name, priority, factory)
#define STEPIT_REGISTER_FIELD_SOURCE(field_name, priority, factory)                                               \
  static auto _field_##field_name##_source_registration = ::stepit::neuro_policy::FieldSourceRegistry::instance() \
                                                              .createRegistration(#field_name, priority, factory)

#endif  // STEPIT_NEURO_POLICY_FIELD_H_

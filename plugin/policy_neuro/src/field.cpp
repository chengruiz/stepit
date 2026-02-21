#include <stepit/policy_neuro/field.h>

namespace stepit {
namespace neuro_policy {
FieldId Module::registerRequirement(const std::string &field_name, FieldSize field_size) {
  return registerRequirement(registerField(field_name, field_size));
}

FieldId Module::registerRequirement(FieldId field_id) {
  if (provisions_.find(field_id) == provisions_.end()) {
    requirements_.insert(field_id);
  }
  return field_id;
}

FieldId Module::registerProvision(const std::string &field_name, FieldSize field_size) {
  FieldId id = registerField(field_name, field_size);
  provisions_.insert(id);
  return id;
}

FieldManager &FieldManager::instance() {
  static FieldManager inst;
  return inst;
}

auto FieldManager::registerSource(const std::string &field_name, int priority, SourceRegistry::Factory factory)
    -> SourceRegistry::Registration {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return source_registry_.createRegistration(field_name, priority, std::move(factory));
}

auto FieldManager::makeSource(const std::string &field_name, const PolicySpec &policy_spec, const std::string &home_dir)
    -> Module::Ptr {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return source_registry_.make(field_name, policy_spec, home_dir);
}

FieldId FieldManager::registerField(const std::string &name, FieldSize size) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto it = name_to_id_.find(name);
  if (it == name_to_id_.end()) {  // If not registered
    auto id           = next_id_++;
    name_to_id_[name] = id;
    id_to_name_.push_back(name);
    id_to_size_.push_back(size);
    return id;
  }

  auto id = it->second;
  if (size != 0) setFieldSize(id, size);
  return id;
}

FieldId FieldManager::getFieldId(const std::string &name) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto it = name_to_id_.find(name);
  STEPIT_ASSERT(it != name_to_id_.end(), "Unregistered observation: '{}'.", name);
  return it->second;
}

const std::string &FieldManager::getFieldName(FieldId id) const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return id_to_name_[id];
}

FieldSize FieldManager::getFieldSize(FieldId id) const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  FieldSize size = id_to_size_[id];
  STEPIT_ASSERT(size > 0, "Size of field '{}' is undefined.", getFieldName(id));
  return size;
}

void FieldManager::setFieldSize(FieldId id, FieldSize size) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto registered_size = id_to_size_.at(id);
  if (registered_size == 0) {  // If not registered
    id_to_size_[id] = size;
    return;
  }
  STEPIT_ASSERT(registered_size == size,
                "Attempting to register field '{}' with size {}, which is already registered with size {}.",
                getFieldName(id), size, registered_size);
}

void parseFieldIds(const YAML::Node &node, FieldIdVec &context) {
  STEPIT_ASSERT(node.IsSequence(), "Expected sequence node for field IDs.");
  for (const auto &item : node) {
    auto name  = item.as<std::string>();
    FieldId id = getFieldId(name);
    context.push_back(id);
  }
}

void stackField(cArrXf vec, uint32_t &offset, rArrXf result) {
  STEPIT_ASSERT(offset + vec.size() <= result.size(), "Field segment size ({} + {}) out of bounds ({}).", offset,
                vec.size(), result.size());
  result.segment(offset, vec.size()) = vec;
  offset += vec.size();
}

void concatFields(const FieldMap &context, const FieldIdVec &field_ids, rArrXf result) {
  uint32_t offset = 0;
  for (auto field_id : field_ids) {
    stackField(context.at(field_id), offset, result);
  }
  STEPIT_ASSERT(offset == result.size(), "Concat field size ({}) does not match the result size ({}).", offset,
                result.size());
}

void splitFields(cArrXf source, const FieldIdVec &field_ids, FieldMap &context) {
  FieldSize offset = 0;
  for (auto field_id : field_ids) {
    FieldSize size = getFieldSize(field_id);
    STEPIT_ASSERT(offset + size <= source.size(), "Field segment size ({} + {}) out of bound ({}).", offset, size,
                  source.size());
    context[field_id] = source.segment(offset, size);
    offset += size;
  }
  STEPIT_ASSERT(offset == source.size(), "Split field size ({}) does not match the source size ({}).", offset,
                source.size());
}
}  // namespace neuro_policy
}  // namespace stepit

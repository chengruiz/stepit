#include <stepit/policy_neuro/field.h>

namespace stepit {
namespace neuro_policy {
FieldId Module::registerRequirement(const std::string &field_name) {
  return registerRequirement(registerField(field_name, 0));
}

FieldId Module::registerRequirement(FieldId field_id) {
  requirements_.insert(field_id);
  return field_id;
}

FieldId Module::registerProvision(const std::string &field_name, std::uint32_t size) {
  FieldId id = registerField(field_name, size);
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

FieldId FieldManager::registerField(const std::string &name, std::uint32_t size) {
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

std::uint32_t FieldManager::getFieldSize(FieldId id) const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return id_to_size_[id];
}

void FieldManager::setFieldSize(FieldId id, std::uint32_t size) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto registered_size = id_to_size_.at(id);
  if (registered_size == 0) {  // If not registered
    id_to_size_[id] = size;
    return;
  }
  if (registered_size != size) {
    STEPIT_ERROR("Attempting to register field '{}' with size {}, which is already registered with size {}.",
                 getFieldName(id), size, registered_size);
  }
}

void parseFieldIds(const YAML::Node &node, FieldIdVec &result) {
  STEPIT_ASSERT(node.IsSequence(), "Expected sequence node for field IDs.");
  for (const auto &item : node) {
    auto name  = item.as<std::string>();
    FieldId id = getFieldId(name);
    result.push_back(id);
  }
}

void stackField(cArrXf vec, uint32_t &index, rArrXf result) {
  STEPIT_ASSERT(index + vec.size() <= result.size(), "Field segment size ({}+{}) out of bounds ({}).", index,
                vec.size(), result.size());
  result.segment(index, vec.size()) = vec;
  index += vec.size();
}

void assembleFields(const FieldMap &field_map, const FieldIdVec &field_ids, rArrXf result) {
  uint32_t index = 0;
  for (auto field_id : field_ids) {
    stackField(field_map.at(field_id), index, result);
  }
  STEPIT_ASSERT(index == result.size(), "Assembled field size ({}) does not match the result size ({}).", index,
                result.size());
}

void splitFields(cArrXf data, const FieldIdVec &field_ids, FieldMap &result) {
  std::uint32_t index = 0;
  for (auto field_id : field_ids) {
    std::uint32_t size = getFieldSize(field_id);
    STEPIT_ASSERT(size > 0, "Size of '{}' is undefined.", getFieldName(field_id));
    result[field_id] = data.segment(index, size);
    index += size;
  }
}
}  // namespace neuro_policy
}  // namespace stepit

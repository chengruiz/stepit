#ifndef STEPIT_NEURO_POLICY_REDIS_FIELD_SUBSCRIBER_H_
#define STEPIT_NEURO_POLICY_REDIS_FIELD_SUBSCRIBER_H_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <stepit/policy_neuro/module.h>
#include <stepit/redis/redis_client.h>

namespace stepit {
namespace neuro_policy {
class RedisFieldSubscriber : public Module {
 public:
  RedisFieldSubscriber(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;
  void exit() override;

 private:
  struct FieldData {
    FieldId id{};
    std::string name;
    std::string key;
    std::string json_field;
    std::size_t size{};
    float timeout_threshold{};

    bool received{false};
    TimePoint stamp{};
    VecXf data;
  };

  void parseFields();
  void addField(const yml::Node &key_node, const yml::Node &value_node);

  bool fetchFields();
  redis::RedisReadStatus fetchField(FieldData &field);

  bool parseFieldValue(const FieldData &field, const redis::RedisClient::JsonDict &payload, VecXf &data) const;
  std::string formatRedisField(const FieldData &field) const;

  redis::RedisClientConfig connection_;
  std::vector<FieldData> fields_;
  std::unique_ptr<redis::RedisClient> redis_client_;
  std::mutex mutex_;
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_REDIS_FIELD_SUBSCRIBER_H_

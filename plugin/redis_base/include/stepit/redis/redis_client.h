#ifndef STEPIT_REDIS_CLIENT_H_
#define STEPIT_REDIS_CLIENT_H_

#include <memory>
#include <mutex>
#include <string>

#include <hiredis/hiredis.h>
#include <nlohmann/json.hpp>

#include <stepit/utils.h>

namespace stepit {
namespace redis {
struct RedisClientConfig {
  RedisClientConfig() = default;
  explicit RedisClientConfig(const yml::Node &node);

  std::string host{"127.0.0.1"};
  int port{6379};
  int db{0};
  std::string username;
  std::string password;
  int connect_timeout_ms{1000};
  int command_timeout_ms{50};
};

RedisClientConfig &getDefaultRedisClientConfig();

enum class RedisReadStatus {
  kOk,
  kMissing,
  kInvalidData,
  kError,
};

class RedisClient {
 public:
  using JsonDict = nlohmann::json;

  RedisClient(std::string label, RedisClientConfig config);

  void disconnect();

  RedisReadStatus get(const std::string &key, JsonDict &value);

 private:
  using RedisContextPtr = std::unique_ptr<redisContext, decltype(&redisFree)>;
  using RedisReplyPtr   = std::unique_ptr<redisReply, decltype(&freeReplyObject)>;

  bool connect();
  bool authenticate();
  bool selectDb();

  RedisReadStatus readValue(const std::string &key, std::string &value);
  RedisReplyPtr runCommand(int argc, const char **argv, const size_t *argvlen);
  RedisReplyPtr runCommand(const std::string &command, const std::string &arg1);
  RedisReplyPtr runCommand(const std::string &command, const std::string &arg1, const std::string &arg2);
  void reportError(const std::string &message);
  void clearError();

  std::string label_;
  RedisClientConfig config_;
  RedisContextPtr redis_{nullptr, &redisFree};
  std::string last_error_;
  std::mutex mutex_;
};

RedisClient &getDefaultRedisClient();
}  // namespace redis
}  // namespace stepit

#endif  // STEPIT_REDIS_CLIENT_H_

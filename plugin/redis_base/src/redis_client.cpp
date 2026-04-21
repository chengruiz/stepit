#include <stepit/redis/redis_client.h>

#define STEPIT_HIREDIS_VERSION_AT_LEAST(major, minor, patch) \
  ((HIREDIS_MAJOR > (major)) or                              \
   (HIREDIS_MAJOR == (major) and                             \
    (HIREDIS_MINOR > (minor) or (HIREDIS_MINOR == (minor) and HIREDIS_PATCH >= (patch)))))

namespace stepit {
namespace redis {
namespace {
timeval makeTimeval(int timeout_ms) {
  timeval value{};
  value.tv_sec  = timeout_ms / 1000;
  value.tv_usec = (timeout_ms % 1000) * 1000;
  return value;
}

bool isSupportedReplyType(int type) {
  switch (type) {
    case REDIS_REPLY_STRING:
    case REDIS_REPLY_STATUS:
    case REDIS_REPLY_INTEGER:
#ifdef REDIS_REPLY_DOUBLE
    case REDIS_REPLY_DOUBLE:
#endif
      return true;
    default:
      return false;
  }
}

std::string getReplyText(const redisReply &reply) {
  switch (reply.type) {
    case REDIS_REPLY_STRING:
    case REDIS_REPLY_STATUS:
    case REDIS_REPLY_ERROR:
      return reply.str == nullptr ? std::string() : std::string(reply.str, reply.len);
    case REDIS_REPLY_INTEGER:
      return std::to_string(reply.integer);
#ifdef REDIS_REPLY_DOUBLE
    case REDIS_REPLY_DOUBLE:
      return reply.str == nullptr ? std::string() : std::string(reply.str, reply.len);
#endif
    default:
      return std::string();
  }
}

std::string getContextErrorSuffix(const redisContext *context) {
  if (context == nullptr or context->errstr[0] == '\0') return {};
  return fmt::format(": {}", context->errstr);
}
}  // namespace

RedisClientConfig::RedisClientConfig(const yml::Node &node) {
  node.assertMap();
  node["host"].to(host, true);
  node["port"].to(port, true);
  node["db"].to(db, true);
  node["username"].to(username, true);
  node["password"].to(password, true);
  node["connect_timeout_ms"].to(connect_timeout_ms, true);
  node["command_timeout_ms"].to(command_timeout_ms, true);
}

RedisClient::RedisClient(std::string label, RedisClientConfig config) : label_(std::move(label)), config_(config) {
  STEPIT_ASSERT(config_.port > 0, "Redis port must be positive, got {}.", config_.port);
  STEPIT_ASSERT(config_.db >= 0, "Redis db index must be non-negative, got {}.", config_.db);
  STEPIT_ASSERT(config_.connect_timeout_ms >= 0, "'connect_timeout_ms' must be non-negative, got {}.",
                config_.connect_timeout_ms);
  STEPIT_ASSERT(config_.command_timeout_ms >= 0, "'command_timeout_ms' must be non-negative, got {}.",
                config_.command_timeout_ms);
}

void RedisClient::disconnect() { redis_.reset(); }

RedisReadStatus RedisClient::get(const std::string &key, JsonDict &value) {
  std::string raw_value;
  RedisReadStatus status = readValue(key, raw_value);
  if (status != RedisReadStatus::kOk) return status;

  try {
    value = JsonDict::parse(raw_value);
  } catch (const JsonDict::exception &err) {
    reportError(fmt::format("Redis key '{}' does not contain valid JSON: {}.", key, err.what()));
    return RedisReadStatus::kInvalidData;
  }

  if (not value.is_object()) {
    reportError(fmt::format("Redis key '{}' must contain a JSON object, got {}.", key, value.type_name()));
    return RedisReadStatus::kInvalidData;
  }

  clearError();
  return RedisReadStatus::kOk;
}

bool RedisClient::connect() {
  if (redis_ != nullptr and redis_->err == REDIS_OK) return true;

  RedisContextPtr context(nullptr, &redisFree);

#if STEPIT_HIREDIS_VERSION_AT_LEAST(1, 0, 0)
  redisOptions options{};
  REDIS_OPTIONS_SET_TCP(&options, config_.host.c_str(), config_.port);

  timeval connect_timeout{};
  if (config_.connect_timeout_ms > 0) {
    connect_timeout         = makeTimeval(config_.connect_timeout_ms);
    options.connect_timeout = &connect_timeout;
  }

  timeval command_timeout{};
  if (config_.command_timeout_ms > 0) {
    command_timeout         = makeTimeval(config_.command_timeout_ms);
    options.command_timeout = &command_timeout;
  }

  context.reset(redisConnectWithOptions(&options));
#else
  if (config_.connect_timeout_ms > 0) {
    const timeval connect_timeout = makeTimeval(config_.connect_timeout_ms);
    context.reset(redisConnectWithTimeout(config_.host.c_str(), config_.port, connect_timeout));
  } else {
    context.reset(redisConnect(config_.host.c_str(), config_.port));
  }

  if (context != nullptr and context->err == REDIS_OK and config_.command_timeout_ms > 0) {
    const timeval command_timeout = makeTimeval(config_.command_timeout_ms);
    if (redisSetTimeout(context.get(), command_timeout) != REDIS_OK) {
      reportError(fmt::format("Failed to set Redis command timeout for {}:{}{}.", config_.host, config_.port,
                              getContextErrorSuffix(context.get())));
      return false;
    }
  }
#endif

  if (not context) {
    reportError(fmt::format("Failed to connect to Redis at {}:{}.", config_.host, config_.port));
    return false;
  }
  if (context->err != REDIS_OK) {
    reportError(fmt::format("Failed to connect to Redis at {}:{}: {}.", config_.host, config_.port, context->errstr));
    return false;
  }

  redis_ = std::move(context);
  if (not authenticate()) {
    disconnect();
    return false;
  }
  if (not selectDb()) {
    disconnect();
    return false;
  }

  clearError();
  return true;
}

bool RedisClient::authenticate() {
  if (config_.password.empty()) return true;

  RedisReplyPtr reply = config_.username.empty() ? runCommand("AUTH", config_.password)
                                                 : runCommand("AUTH", config_.username, config_.password);
  if (not reply) {
    reportError(fmt::format("Redis AUTH failed{}.", getContextErrorSuffix(redis_.get())));
    return false;
  }
  if (reply->type == REDIS_REPLY_ERROR) {
    reportError(fmt::format("Redis AUTH failed: {}.", getReplyText(*reply)));
    return false;
  }
  if (reply->type != REDIS_REPLY_STATUS or getReplyText(*reply) != "OK") {
    reportError(fmt::format("Redis AUTH returned unexpected reply: {}.", getReplyText(*reply)));
    return false;
  }
  return true;
}

bool RedisClient::selectDb() {
  if (config_.db == 0) return true;

  RedisReplyPtr reply = runCommand("SELECT", std::to_string(config_.db));
  if (not reply) {
    reportError(fmt::format("Redis SELECT {} failed{}.", config_.db, getContextErrorSuffix(redis_.get())));
    return false;
  }
  if (reply->type == REDIS_REPLY_ERROR) {
    reportError(fmt::format("Redis SELECT {} failed: {}.", config_.db, getReplyText(*reply)));
    return false;
  }
  if (reply->type != REDIS_REPLY_STATUS or getReplyText(*reply) != "OK") {
    reportError(fmt::format("Redis SELECT {} returned unexpected reply: {}.", config_.db, getReplyText(*reply)));
    return false;
  }
  return true;
}

RedisReadStatus RedisClient::readValue(const std::string &key, std::string &value) {
  if (not connect()) return RedisReadStatus::kError;

  RedisReplyPtr reply = runCommand("GET", key);
  if (not reply) {
    reportError(fmt::format("Redis read failed for '{}'{}.", key, getContextErrorSuffix(redis_.get())));
    disconnect();
    return RedisReadStatus::kError;
  }
  if (reply->type == REDIS_REPLY_NIL) return RedisReadStatus::kMissing;

  if (reply->type == REDIS_REPLY_ERROR) {
    reportError(fmt::format("Redis read failed for '{}': {}.", key, getReplyText(*reply)));
    return RedisReadStatus::kError;
  }
  if (not isSupportedReplyType(reply->type)) {
    reportError(fmt::format("Redis read for '{}' returned unsupported reply type {}.", key, reply->type));
    return RedisReadStatus::kError;
  }

  value = getReplyText(*reply);
  clearError();
  return RedisReadStatus::kOk;
}

RedisClient::RedisReplyPtr RedisClient::runCommand(int argc, const char **argv, const size_t *argvlen) {
  if (redis_ == nullptr) return RedisReplyPtr(nullptr, &freeReplyObject);
  return RedisReplyPtr(static_cast<redisReply *>(redisCommandArgv(redis_.get(), argc, argv, argvlen)),
                       &freeReplyObject);
}

RedisClient::RedisReplyPtr RedisClient::runCommand(const std::string &command, const std::string &arg1) {
  const char *argv[] = {command.c_str(), arg1.c_str()};
  size_t argvlen[]   = {command.size(), arg1.size()};
  return runCommand(2, argv, argvlen);
}

RedisClient::RedisReplyPtr RedisClient::runCommand(const std::string &command, const std::string &arg1,
                                                   const std::string &arg2) {
  const char *argv[] = {command.c_str(), arg1.c_str(), arg2.c_str()};
  size_t argvlen[]   = {command.size(), arg1.size(), arg2.size()};
  return runCommand(3, argv, argvlen);
}

void RedisClient::reportError(const std::string &message) {
  if (message == last_error_) return;
  last_error_ = message;
  STEPIT_WARN("Redis client '{}': {}", label_, message);
}

void RedisClient::clearError() {
  if (last_error_.empty()) return;
  STEPIT_INFO("Redis client '{}' connected to {}:{}.", label_, config_.host, config_.port);
  last_error_.clear();
}
}  // namespace redis
}  // namespace stepit

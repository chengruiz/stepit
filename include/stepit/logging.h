#ifndef STEPIT_LOGGING_H_
#define STEPIT_LOGGING_H_

#include <mutex>
#include <string>

#include <llu/error.h>

namespace stepit {
enum VerbosityLevel : std::uint8_t {
  kCrit = 0,
  kWarn = 1,
  kInfo = 2,
  kDbug = 3,
};

class LoggingModule {
 public:
  template <typename... Args>
  void operator()(const std::string &format, Args &&...args) {
    if (verbosity_ < next_verbosity_) {
      clearStyle();
      return;
    }
    if (sizeof...(args) == 0) {
      logImpl(format);
    } else {
      logImpl(fmt::format(format, args...));
    }
  }

  std::recursive_mutex &mutex() const { return mutex_; }
  VerbosityLevel getVerbosity() const {
    std::lock_guard<std::recursive_mutex> lock(mutex());
    return verbosity_;
  }
  LoggingModule &setVerbosity(VerbosityLevel level);

  LoggingModule &withoutTimestamp();
  LoggingModule &setNextVerbosity(VerbosityLevel level);
  LoggingModule &setNextTextStyle(const char *style);
  void clearStyle();

  static LoggingModule &instance();

 private:
  void logImpl(const std::string &info);

  mutable std::recursive_mutex mutex_;
  VerbosityLevel verbosity_{kInfo};

  VerbosityLevel next_verbosity_{kInfo};
  bool next_timestamp_{true};
  std::string next_text_style_;
};
}  // namespace stepit

#define STEPIT_LOG_INSTANCE ::stepit::LoggingModule::instance()
#define STEPIT_VERBOSITY    STEPIT_LOG_INSTANCE.getVerbosity()
#define STEPIT_LOG_CALL(...)                                                 \
  do {                                                                       \
    std::lock_guard<std::recursive_mutex> lock(STEPIT_LOG_INSTANCE.mutex()); \
    STEPIT_LOG_INSTANCE __VA_ARGS__;                                         \
  } while (0)
#define STEPIT_SET_VERBOSITY(verbosity) STEPIT_LOG_CALL(.setVerbosity(verbosity))
#define STEPIT_LOG_IMPL(format, ...)    STEPIT_LOG_CALL(format(__VA_ARGS__))

#define STEPIT_LOG_FORMAT_CRIT .setNextVerbosity(::stepit::kCrit).setNextTextStyle(::llu::kRed)
#define STEPIT_LOG_FORMAT_WARN .setNextVerbosity(::stepit::kWarn).setNextTextStyle(::llu::kYellow)
#define STEPIT_LOG_FORMAT_INFO .setNextVerbosity(::stepit::kInfo).setNextTextStyle(::llu::kGreen)
#define STEPIT_LOG_FORMAT_DBUG .setNextVerbosity(::stepit::kDbug)
#define STEPIT_LOG_FORMAT_NT   .withoutTimestamp()

/* Logging macros */
#define STEPIT_LOG(...)    STEPIT_LOG_IMPL(, __VA_ARGS__)
#define STEPIT_LOGNT(...)  STEPIT_LOG_IMPL(STEPIT_LOG_FORMAT_NT, __VA_ARGS__)
#define STEPIT_CRIT(...)   STEPIT_LOG_IMPL(STEPIT_LOG_FORMAT_CRIT, __VA_ARGS__)
#define STEPIT_WARN(...)   STEPIT_LOG_IMPL(STEPIT_LOG_FORMAT_WARN, __VA_ARGS__)
#define STEPIT_INFO(...)   STEPIT_LOG_IMPL(STEPIT_LOG_FORMAT_INFO, __VA_ARGS__)
#define STEPIT_DBUG(...)   STEPIT_LOG_IMPL(STEPIT_LOG_FORMAT_DBUG, __VA_ARGS__)
#define STEPIT_CRITNT(...) STEPIT_LOG_IMPL(STEPIT_LOG_FORMAT_CRIT STEPIT_LOG_FORMAT_NT, __VA_ARGS__)
#define STEPIT_WARNNT(...) STEPIT_LOG_IMPL(STEPIT_LOG_FORMAT_WARN STEPIT_LOG_FORMAT_NT, __VA_ARGS__)
#define STEPIT_INFONT(...) STEPIT_LOG_IMPL(STEPIT_LOG_FORMAT_INFO STEPIT_LOG_FORMAT_NT, __VA_ARGS__)
#define STEPIT_DBUGNT(...) STEPIT_LOG_IMPL(STEPIT_LOG_FORMAT_DBUG STEPIT_LOG_FORMAT_NT, __VA_ARGS__)

#define STEPIT_THROW         LLU_THROW
#define STEPIT_ASSERT        LLU_ASSERT
#define STEPIT_ASSERT_EQ     LLU_ASSERT_EQ
#define STEPIT_UNREACHABLE() STEPIT_THROW("Unreachable code reached.")

namespace stepit {
std::string getCurrentTimeStamp(const char *format = "%F %T", bool milliseconds = true);
void displayFormattedBanner(std::size_t width, const char *style = nullptr, std::string msg = "");

template <typename... Args>
void displayFormattedBanner(std::size_t width, const char *style, const std::string &format, Args &&...args) {
  displayFormattedBanner(width, style, fmt::format(format, std::forward<Args>(args)...));
}
}  // namespace stepit

#endif  // STEPIT_LOGGING_H_

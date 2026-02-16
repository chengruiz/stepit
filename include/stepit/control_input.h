#ifndef STEPIT_CONTROL_INPUT_H_
#define STEPIT_CONTROL_INPUT_H_

#include <cstdio>
#include <deque>
#include <future>
#include <list>
#include <mutex>
#include <string>

#include <boost/optional.hpp>

#include <stepit/registry.h>

namespace stepit {
enum ErrorCode : std::uint8_t {
  kSuccess             = 0,
  kNotInCorrectState   = 1,
  kPolicyNotFound      = 2,
  kIncorrectArgument   = 3,
  kUnrecognizedRequest = 4,
  kInvalidRequest      = 255,
};

constexpr const char *kAgentNotInCorrectState = "Agent is not in correct state.";

struct ControlResponse {
  std::uint8_t status{kInvalidRequest};
  std::string message;
};

struct ControlRequest {
  explicit ControlRequest(const std::string &request_str = "");
  ControlRequest(const ControlRequest &other)                = delete;
  ControlRequest &operator=(const ControlRequest &other)     = delete;
  ControlRequest(ControlRequest &&other) noexcept            = default;
  ControlRequest &operator=(ControlRequest &&other) noexcept = default;
  const std::string &string() const { return string_; }
  const std::string &channel() const { return channel_; }
  const std::string &action() const { return action_; }
  const std::string &argument() const { return argument_; }

  template <typename... Args>
  bool parseArgument(const char *format, Args &...args) const {
    int result = std::sscanf(argument_.c_str(), format, &args...);
    return result == sizeof...(Args);
  }

  std::future<ControlResponse> getResponse() { return response_.get_future(); }
  void response(std::uint8_t code = 0, std::string message = "");
  void response(ErrorCode code, std::string message = "");
  bool channelMatches(const std::string &prefix) const { return channel_.rfind(prefix, 0) == 0; }
  friend std::ostream &operator<<(std::ostream &os, const ControlRequest &request) { return os << request.string_; }

 private:
  std::string string_, command_, channel_, action_, argument_;
  std::promise<ControlResponse> response_;
};

struct ControlRequests : public std::list<ControlRequest> {
  using std::list<ControlRequest>::list;
  ControlRequests filterByChannel(const std::string &channel_prefix);
};

/**
 * @class ControlInput
 * @brief An abstract base class defining the interface for handling control inputs.
 *
 * The ControlInput class serves as a standard interface for various input methods
 * (e.g., console, joystick, publisher). It provides mechanisms to poll for new data,
 * check availability, and retrieve control requests in a thread-safe manner.
 */
class ControlInput : public Interface<ControlInput> {
 public:
  /**
   * @brief Checks if there are any control requests available to be processed.
   * 
   * @return true if the internal request queue is not empty, false otherwise.
   */
  virtual bool available() const = 0;

  /**
   * @brief Polls the underlying input source for new data.
   */
  virtual void poll() = 0;

  /**
   * @brief Retrieves and removes the next control request from the queue.
   * 
   * @return A boost::optional containing the next ControlRequest if available,
   *         or boost::none if the queue is empty.
   */
  boost::optional<ControlRequest> pop();

 protected:
  /**
   * @brief Parses a raw string request and pushes it onto the internal queue.
   * 
   * @param request_str The raw string representation of the control command.
   * @return std::future<ControlResponse> A future that will eventually contain the
   *         result of processing this request.
   */
  std::future<ControlResponse> put(std::string request_str);

 private:
  /* Mutex to protect access to the `requests_` deque */
  std::mutex mutex_;
  /* Queue storing pending control requests */
  std::deque<ControlRequest> requests_;
};

class MultipleControlInputs {
 public:
  explicit MultipleControlInputs(const std::vector<std::string> &input_names);
  bool available() const;
  void poll();
  boost::optional<ControlRequest> pop();

 private:
  std::vector<ControlInput::Ptr> inputs_;
};

template <typename Action>
Action lookupAction(const std::string &action, const std::map<std::string, Action> &action_map) {
  return lookupMap(action, action_map, Action::kInvalid);
}
}  // namespace stepit

#if FMT_VERSION >= 90000
template <>
struct fmt::formatter<stepit::ControlRequest> : fmt::ostream_formatter {};
#endif

#define STEPIT_REGISTER_CTRLINPUT(name, priority, factory) \
  static ::stepit::ControlInput::Registration _ctrl_##name##_registration(#name, priority, factory)

#endif  // STEPIT_CONTROL_INPUT_H_

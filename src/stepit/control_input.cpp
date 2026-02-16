#include <set>

#include <stepit/control_input.h>

namespace stepit {
ControlRequest::ControlRequest(const std::string &request_str) : string_(request_str) {
  if (request_str.empty()) return;

  auto action_sep = request_str.find(':');
  if (action_sep == std::string::npos) {
    command_  = request_str;
    argument_ = "";
  } else {
    command_  = request_str.substr(0, action_sep);
    argument_ = request_str.substr(action_sep + 1);
  }

  auto channel_sep = command_.rfind('/');
  if (channel_sep != std::string::npos) {
    channel_ = command_.substr(0, channel_sep);
    action_  = command_.substr(channel_sep + 1);
  }
}

void ControlRequest::response(std::uint8_t code, std::string message) {
  response_.set_value({code, std::move(message)});
}

void ControlRequest::response(ErrorCode code, std::string message) {
  switch (code) {
    case kNotInCorrectState:
      if (message.empty()) message = fmt::format("Not in correct state for request '{}'.", *this);
      STEPIT_DBUG(message);
      break;
    case kPolicyNotFound:
      if (message.empty()) message = "Policy not found.";
      STEPIT_CRIT(message);
      break;
    case kIncorrectArgument:
      if (message.empty()) {
        message = fmt::format("Incorrect argument '{}' for command '{}'.", argument_, command_);
      }
      STEPIT_CRIT(message);
      break;
    case kUnrecognizedRequest:
      if (message.empty()) message = fmt::format("Unrecognized request '{}'.", *this);
      STEPIT_DBUG(message);
      break;
    case kSuccess:
      if (not message.empty()) STEPIT_LOG(message);
      break;
    default:
      break;
  }
  response(static_cast<std::uint8_t>(code), std::move(message));
}

ControlRequests ControlRequests::filterByChannel(const std::string &channel_prefix) {
  ControlRequests result;
  for (auto it = begin(); it != end();) {
    if (it->channelMatches(channel_prefix)) {
      auto next = std::next(it);
      result.splice(result.end(), *this, it);
      it = next;
    } else {
      ++it;
    }
  }
  return result;
}

boost::optional<ControlRequest> ControlInput::pop() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (requests_.empty()) return {};
  ControlRequest request = std::move(requests_.front());
  requests_.pop_front();
  return request;
}

std::future<ControlResponse> ControlInput::put(std::string request_str) {
  std::lock_guard<std::mutex> lock(mutex_);
  requests_.emplace_back(std::move(request_str));
  return requests_.back().getResponse();
}

MultipleControlInputs::MultipleControlInputs(const std::vector<std::string> &input_names) {
  if (input_names.empty()) {
    inputs_.emplace_back(ControlInput::make(""));
  } else {
    std::set<std::string> inputs(input_names.begin(), input_names.end());
    for (const auto &input_name : inputs) {
      inputs_.emplace_back(ControlInput::make(input_name));
    }
  }
}

bool MultipleControlInputs::available() const {
  return std::any_of(inputs_.begin(), inputs_.end(), [](const auto &input) { return input->available(); });
}

void MultipleControlInputs::poll() {
  for (auto &input : inputs_) input->poll();
}

boost::optional<ControlRequest> MultipleControlInputs::pop() {
  for (auto &input : inputs_) {
    auto request = input->pop();
    if (request.has_value()) return request;
  }
  return {};
}
}  // namespace stepit

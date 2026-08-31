#ifndef STEPIT_MODULAR_POLICY_SUBSCRIBER_ACTION_H_
#define STEPIT_MODULAR_POLICY_SUBSCRIBER_ACTION_H_

#include <cstdint>
#include <map>
#include <string>

namespace stepit {
namespace modular_policy {
enum class SubscriberAction : std::uint8_t {
  kEnableSubscriber,
  kDisableSubscriber,
  kSwitchSubscriber,
  kInvalid = 255,
};

extern const std::map<std::string, SubscriberAction> kSubscriberActionMap;

constexpr const char *kStartSubscribingTemplate = "\033[1;32mStarted\033[0m subscribing {}.";
constexpr const char *kStopSubscribingTemplate  = "\033[1;33mStopped\033[0m subscribing {}.";
constexpr const char *kActionBlockedTemplate    = "Action '{}' is blocked when subscriber is enabled.";
}  // namespace modular_policy
}  // namespace stepit

#endif  // STEPIT_MODULAR_POLICY_SUBSCRIBER_ACTION_H_

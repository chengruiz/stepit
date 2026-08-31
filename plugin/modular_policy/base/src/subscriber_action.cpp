#include <stepit/control_input.h>
#include <stepit/modular_policy/subscriber_action.h>

namespace stepit {
namespace modular_policy {
// clang-format off
const std::map<std::string, SubscriberAction> kSubscriberActionMap = {
    {"EnableSubscriber",  SubscriberAction::kEnableSubscriber},
    {"DisableSubscriber", SubscriberAction::kDisableSubscriber},
    {"SwitchSubscriber", SubscriberAction::kSwitchSubscriber},
};
// clang-format on
}  // namespace modular_policy
}  // namespace stepit

#include <stepit/publisher.h>
#include <stepit/spin.h>

namespace stepit {
STEPIT_REGISTER_PUBLISHER(dummy, kMinPriority, std::make_unique<Publisher>);
STEPIT_REGISTER_SPIN(catch_sigint_spin, kMinPriority + 1, std::make_unique<WaitForSigInt>);
}  // namespace stepit

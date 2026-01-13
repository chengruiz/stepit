#include <stepit/publisher.h>

namespace stepit {
Publisher &Publisher::instance() {
  static std::unique_ptr<Publisher> instance_{PublisherReg::make("")};
  return *instance_;
}

namespace publisher {
Filter::Filter() {
  getenv("STEPIT_PUBLISH_STATUS", publish_status);
  getenv("STEPIT_PUBLISH_LOW_LEVEL", publish_low_level);
  getenv("STEPIT_PUBLISH_ARRAY", publish_array);
}

const Filter g_filter;

StatusRegistration::StatusRegistration(const std::string &name) : name_(name) {
  STEPIT_ASSERT(not hasStatus(name_), "Status '{}' is already registered.", name_);
}

StatusRegistration::~StatusRegistration() {
  if (not name_.empty()) removeStatus(name_);
}

StatusRegistration::StatusRegistration(StatusRegistration &&other) noexcept : name_(std::move(other.name_)) {
  other.name_.clear();
}

StatusRegistration &StatusRegistration::operator=(StatusRegistration &&other) noexcept {
  if (this != &other) {
    name_ = std::move(other.name_);
    other.name_.clear();
  }
  return *this;
}
}  // namespace publisher
}  // namespace stepit

#include <stepit/utils.h>
#include <stepit/robot/unitree2/common.h>

namespace stepit {
uint32_t crc32core(const uint32_t *ptr, uint32_t len) {
  constexpr uint32_t dw_polynomial = 0x04c11db7;

  uint32_t xbit  = 0;
  uint32_t data  = 0;
  uint32_t crc32 = 0xFFFFFFFF;
  for (uint32_t i = 0; i < len; i++) {
    xbit = 1 << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++) {
      if (crc32 & 0x80000000) {
        crc32 <<= 1;
        crc32 ^= dw_polynomial;
      } else {
        crc32 <<= 1;
      }
      if (data & xbit) crc32 ^= dw_polynomial;

      xbit >>= 1;
    }
  }
  return crc32;
}

Unitree2ServiceClient &Unitree2ServiceClient::instance() {
  static Unitree2ServiceClient instance;
  return instance;
}

void Unitree2ServiceClient::initialize_() {
  if (initialized_) return;
  getenv("STEPIT_NETIF", network_interface_);
  getenv("STEPIT_UNITREE2_DOMAIN_ID", domain_id_);
  u2_sdk::ChannelFactory::Instance()->Init(static_cast<int32_t>(domain_id_), network_interface_);
  simulated_   = network_interface_ == "lo";
  initialized_ = true;

  if (simulated_) {
    disabled_ = true;
  } else {
    client_ = std::make_unique<u2_sdk::b2::MotionSwitcherClient>();
    client_->SetTimeout(10.0F);
    client_->Init();

    std::string client_version = client_->GetApiVersion();
    std::string server_version = client_->GetServerApiVersion();
    if (client_version != server_version) {
      STEPIT_WARN("Unitree: API version mismatch (client {}, server {}).", client_version, server_version);
    }

    int32_t ret = client_->GetSilent(disabled_);
    STEPIT_ASSERT(ret == 0, "Unitree: GetSilent failed (error code: {}).", ret);
    ret = client_->CheckMode(robot_type_, motion_type_);
    STEPIT_ASSERT(ret == 0, "Unitree: CheckMode failed (error code: {}).", ret);
  }
}

void Unitree2ServiceClient::status_() const {
  STEPIT_ASSERT(initialized_, "Unitree: Service client is not initialized.");
  if (simulated_) return;
  STEPIT_INFO("Unitree: robot type: '{}', motion type: '{}'.", robot_type_, motion_type_);
}

void Unitree2ServiceClient::activate_(const std::string &mode) {
  STEPIT_ASSERT(initialized_, "Unitree: Service client is not initialized.");
  if (simulated_) return;
  if (motion_type_ == mode) {
    STEPIT_LOG("Unitree: Locomotion mode is already '{}'.", mode);
    return;
  }
  int32_t ret = client_->SelectMode(mode);
  STEPIT_ASSERT(ret == 0, "Unitree: SelectMode failed (error code: {}).", ret);
  ret = client_->CheckMode(robot_type_, motion_type_);
  STEPIT_ASSERT(ret == 0, "Unitree: CheckMode failed (error code: {}).", ret);
  STEPIT_LOG("Unitree: Locomotion mode is switched to '{}'.", motion_type_);
}

void Unitree2ServiceClient::deactivate_() {
  STEPIT_ASSERT(initialized_, "Unitree: Service client is not initialized.");
  if (simulated_) return;
  if (motion_type_.empty()) {
    STEPIT_LOG("Unitree: Built-in locomotion is already deactivated.");
    return;
  }
  int32_t ret = client_->ReleaseMode();
  STEPIT_ASSERT(ret == 0, "Unitree: ReleaseMode failed (error code: {}).", ret);
  STEPIT_LOG("Unitree: Built-in locomotion is deactivated.");
}

void Unitree2ServiceClient::disable_() {
  STEPIT_ASSERT(initialized_, "Unitree: Service client is not initialized.");
  if (simulated_) return;
  if (disabled_) {
    STEPIT_LOG("Unitree: Built-in locomotion is already disabled.");
    return;
  }
  int32_t ret = client_->SetSilent(true);
  STEPIT_ASSERT(ret == 0, "Unitree: SetMode failed (error code: {}).", ret);
  STEPIT_LOG("Unitree: Built-in locomotion is disabled.");
  disabled_ = true;
}

void Unitree2ServiceClient::enable_() {
  STEPIT_ASSERT(initialized_, "Unitree: Service client is not initialized.");
  if (simulated_) return;
  if (not disabled_) {
    STEPIT_LOG("Unitree: Built-in locomotion is already enabled.");
    return;
  }
  int32_t ret = client_->SetSilent(false);
  STEPIT_ASSERT(ret == 0, "Unitree: SetMode failed (error code: {}).", ret);
  STEPIT_LOG("Unitree: Built-in locomotion is enabled.");
  disabled_ = false;
}
}  // namespace stepit

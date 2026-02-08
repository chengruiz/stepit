#ifndef STEPIT_ROBOT_UNITREE2_COMMON_H_
#define STEPIT_ROBOT_UNITREE2_COMMON_H_

#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

namespace u2_sdk = unitree::robot;

namespace stepit {
// Utility functions
uint32_t crc32core(const uint32_t *ptr, uint32_t len);
template <typename T>
void fillLowCmdCrc(T &low_cmd) {
  low_cmd.crc() = crc32core(reinterpret_cast<uint32_t *>(&low_cmd), (sizeof(T) >> 2) - 1);
}

constexpr float kPosStop             = 2.146E+9F;
constexpr float kVelStop             = 16000.0F;
constexpr uint8_t kGo2MotorServoMode = 0x01;
constexpr uint8_t kB2MotorServoMode  = 0x0A;

class Unitree2ServiceClient {
 public:
  Unitree2ServiceClient(const Unitree2ServiceClient &)            = delete;
  Unitree2ServiceClient &operator=(const Unitree2ServiceClient &) = delete;
  static void initialize() { instance().initialize_(); }
  static void status() { instance().status_(); }
  static void activate(const std::string &mode) { instance().activate_(mode); }
  static void deactivate() { instance().deactivate_(); }
  static void disable() { instance().disable_(); }
  static void enable() { instance().enable_(); }

 private:
  Unitree2ServiceClient() = default;
  void initialize_();
  void status_() const;
  void activate_(const std::string &mode);
  void deactivate_();
  void disable_();
  void enable_();
  static Unitree2ServiceClient &instance();

  bool initialized_{false};
  bool simulated_{false};
  bool disabled_{false};
  long domain_id_{0};
  std::string network_interface_{"eth0"};
  std::unique_ptr<u2_sdk::b2::MotionSwitcherClient> client_;
  std::string robot_type_, motion_type_;
};
}  // namespace stepit

#endif  // STEPIT_ROBOT_UNITREE2_COMMON_H_

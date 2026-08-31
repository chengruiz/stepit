#ifndef STEPIT_ROBOT_UNITREE2_COMMON_H_
#define STEPIT_ROBOT_UNITREE2_COMMON_H_

#include <cstdint>
#include <memory>
#include <string>

#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/go2/robot_state/robot_state_client.hpp>

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

class Unitree2Dds {
 public:
  Unitree2Dds(const Unitree2Dds &)            = delete;
  Unitree2Dds &operator=(const Unitree2Dds &) = delete;
  static void initialize() { instance().initialize_(); }
  static bool isSimulated() { return instance().isSimulated_(); }

 private:
  Unitree2Dds() = default;
  void initialize_();
  bool isSimulated_() const;
  static Unitree2Dds &instance();

  bool initialized_{false};
  bool simulated_{false};
  long domain_id_{0};
  std::string network_interface_{""};
};

class Unitree2MotionSwitcher {
 public:
  Unitree2MotionSwitcher(const Unitree2MotionSwitcher &)            = delete;
  Unitree2MotionSwitcher &operator=(const Unitree2MotionSwitcher &) = delete;
  static void initialize() { instance().initialize_(); }
  static void status() { instance().status_(); }
  static void activate(const std::string &mode) { instance().activate_(mode); }
  static void deactivate() { instance().deactivate_(); }
  static void disable() { instance().disable_(); }
  static void enable() { instance().enable_(); }

 private:
  Unitree2MotionSwitcher() = default;
  void initialize_();
  void status_();
  void activate_(const std::string &mode);
  void deactivate_();
  void disable_();
  void enable_();
  static Unitree2MotionSwitcher &instance();

  bool initialized_{false};
  bool disabled_{false};
  std::unique_ptr<u2_sdk::b2::MotionSwitcherClient> client_;
  std::string robot_type_, motion_type_;
};

class Unitree2ServiceSwitcher {
 public:
  Unitree2ServiceSwitcher(const Unitree2ServiceSwitcher &)            = delete;
  Unitree2ServiceSwitcher &operator=(const Unitree2ServiceSwitcher &) = delete;
  static void initialize() { instance().initialize_(); }
  static void serviceSwitch(const std::string &name, bool enable) { instance().serviceSwitch_(name, enable); }

 private:
  Unitree2ServiceSwitcher() = default;
  void initialize_();
  void serviceSwitch_(const std::string &name, bool enable);
  static Unitree2ServiceSwitcher &instance();

  bool initialized_{false};
  std::unique_ptr<u2_sdk::go2::RobotStateClient> client_;
};
}  // namespace stepit

#endif  // STEPIT_ROBOT_UNITREE2_COMMON_H_

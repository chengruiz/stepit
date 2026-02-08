#ifndef STEPIT_NEURO_POLICY_CMD_VEL_SOURCE_H_
#define STEPIT_NEURO_POLICY_CMD_VEL_SOURCE_H_

#include <map>
#include <string>

#include <stepit/joystick/joystick.h>
#include <stepit/policy_neuro/field.h>

namespace stepit {
namespace neuro_policy {
class CmdVelSource : public Module {
 public:
  CmdVelSource(const PolicySpec &policy_spec, const std::string &home_dir);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &result) override;
  void exit() override;

  enum class Action : std::uint8_t {
    kSetVelocity,
    kSetVelocityUnscaled,
    kSetTurboRatio,
    kSelectMode,
    kCycleMode,
    kEnableSmoothing,
    kDisableSmoothing,
    kSetMaxAccel,
    kEnableJoystick,
    kDisableJoystick,
    kInvalid = 255,
  };

 protected:
  static const std::map<std::string, Action> kActionMap;
  virtual void handleControlRequest(ControlRequest request);

  YAML::Node config_;
  FieldId cmd_vel_id_{};
  FieldId cmd_stall_id_{};
  std::vector<JoystickRule> joystick_rules_;

  Arr3f velocity_scale_factor_{1.0, 0.5, 1.0};
  Arr3f velocity_turbo_factor_{Arr3f::Ones()};
  float velocity_deadzone_{0.1};
  bool smoothing_{false};
  float timestep_{0.01};
  Arr3f max_acceleration_{5., 2.5, 10.};
  bool joystick_enabled_{true};

  enum Mode { kAuto, kStall, kMove, kNumModes } mode_{kAuto};
  static constexpr std::array<const char *, kNumModes> kModeName{"auto", "stall", "move"};
  std::array<bool, kNumModes> mode_enabled_{true, true, true};

  Arr3f cmd_vel_{Arr3f::Zero()};
  Arr3f target_cmd_vel_{Arr3f::Zero()};
  bool cmd_stall_{false};
  double velocity_turbo_ratio_{0.0};
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_CMD_VEL_SOURCE_H_

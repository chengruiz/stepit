#ifndef STEPIT_NEURO_POLICY_CMD_POSTURE_SOURCE_H_
#define STEPIT_NEURO_POLICY_CMD_POSTURE_SOURCE_H_

#include <stepit/policy_neuro/cmd_vel_source.h>
#include <stepit/policy_neuro/field.h>

namespace stepit {
namespace neuro_policy {
class CmdRollSource : public Module {
 public:
  CmdRollSource(const PolicySpec &robot_spec, const std::string &home_dir);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &result) override;
  void exit() override;

  enum class Action : std::uint8_t {
    kSetRoll,
    kSetRollUnscaled,
    kEnableJoystick,
    kDisableJoystick,
    kInvalid = 255,
  };

 protected:
  static const std::map<std::string, Action> kActionMap;
  virtual void handleControlRequest(ControlRequest request);

  YAML::Node config_;
  FieldId cmd_roll_id_{};
  std::vector<JoystickRule> joystick_rules_;

  float roll_scale_factor_{M_PI / 6};
  bool joystick_enabled_{true};

  float cmd_roll_{};
};

class CmdPitchSource : public Module {
 public:
  CmdPitchSource(const PolicySpec &robot_spec, const std::string &home_dir);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &result) override;
  void exit() override;

  enum class Action : std::uint8_t {
    kSetPitch,
    kSetPitchUnscaled,
    kEnableJoystick,
    kDisableJoystick,
    kInvalid = 255,
  };

 protected:
  static const std::map<std::string, Action> kActionMap;
  virtual void handleControlRequest(ControlRequest request);

  YAML::Node config_;
  FieldId cmd_pitch_id_{};
  std::vector<JoystickRule> joystick_rules_;

  float pitch_scale_factor_{M_PI / 6};
  bool joystick_enabled_{true};

  float cmd_pitch_{};
};

class CmdHeightSource : public Module {
 public:
  CmdHeightSource(const PolicySpec &robot_spec, const std::string &home_dir);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &result) override;
  void exit() override;

  enum class Action : std::uint8_t {
    kSetHeight,
    kIncreaseHeight,
    kDecreaseHeight,
    kEnableJoystick,
    kDisableJoystick,
    kInvalid = 255,
  };

 protected:
  static const std::map<std::string, Action> kActionMap;
  virtual void handleControlRequest(ControlRequest request);

  YAML::Node config_;
  FieldId cmd_height_id_{};
  std::vector<JoystickRule> joystick_rules_;

  float default_cmd_height_{1.0F};
  float height_scale_factor_{0.05};
  range_t<float> height_range_{0.6, 1.2};
  bool joystick_enabled_{true};

  float cmd_height_{};
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_CMD_POSTURE_SOURCE_H_

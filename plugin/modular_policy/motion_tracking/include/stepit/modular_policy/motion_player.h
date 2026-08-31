#ifndef STEPIT_MODULAR_POLICY_MOTION_PLAYER_H_
#define STEPIT_MODULAR_POLICY_MOTION_PLAYER_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <stepit/joystick/joystick.h>
#include <stepit/modular_policy/module.h>
#include <stepit/modular_policy/motion_clip.h>

namespace stepit {
namespace modular_policy {
class MotionPlayer : public Module {
 public:
  MotionPlayer(const ModularPolicySpec &policy_spec, const ModuleSpec &module_spec);

  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;
  void exit() override;

  enum class Action : std::uint8_t {
    kReplayCurrentClip,
    kSelectNextClip,
    kSelectPreviousClip,
    kInvalid = 255,
  };

 private:
  struct MotionData {
    std::string name;
    std::vector<std::vector<ArrXf>> fields;
    std::size_t num_frames{};
  };

  struct FieldSpec {
    std::string name;
    std::string key;
    std::string type;
    bool differentiate{false};
    bool zero_at_end{false};
    yml::Indices indices;
    std::vector<std::int64_t> offsets;

    std::size_t frame_size{};
    std::size_t field_size{};
    FieldId field_id{kInvalidFieldId};
  };

  static const std::map<std::string, Action> kActionMap;
  void handleControlRequest(ControlRequest request);

  std::vector<MotionData> motions_;
  std::vector<FieldSpec> field_specs_;
  std::vector<ArrXf> buffers_;
  std::vector<JoystickRule> joystick_rules_;
  FieldId motion_frame_index_id_{};
  FieldId motion_restart_event_id_{};

  std::size_t clip_index_{};
  std::size_t frame_index_{};
  bool motion_restart_event_{true};
};
}  // namespace modular_policy
}  // namespace stepit

#endif  // STEPIT_MODULAR_POLICY_MOTION_PLAYER_H_

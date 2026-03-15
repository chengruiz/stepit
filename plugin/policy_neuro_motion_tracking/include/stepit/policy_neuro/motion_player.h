#ifndef STEPIT_NEURO_POLICY_MOTION_PLAYER_H_
#define STEPIT_NEURO_POLICY_MOTION_PLAYER_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <stepit/joystick/joystick.h>
#include <stepit/policy_neuro/module.h>
#include <stepit/policy_neuro/motion_clip.h>

namespace stepit {
namespace neuro_policy {
class MotionPlayer : public Module {
 public:
  MotionPlayer(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec);

  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;
  void exit() override;

  enum class Action : std::uint8_t {
    kSelectNextClip,
    kReplayCurrentClip,
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

  std::size_t clip_index_{};
  std::size_t frame_index_{};
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_MOTION_PLAYER_H_

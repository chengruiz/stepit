#ifndef STEPIT_NEURO_POLICY_MOTION_TRAJECTORY_H_
#define STEPIT_NEURO_POLICY_MOTION_TRAJECTORY_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <stepit/field/data_loader.h>
#include <stepit/policy_neuro/module.h>

namespace stepit {
namespace neuro_policy {
class MotionTrajectory : public Module {
 public:
  MotionTrajectory(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec);

  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;

 private:
  struct FieldView {
    std::vector<ArrXf> source;
    std::size_t frame_size{};
    std::size_t field_size{};
    std::vector<std::int64_t> offsets;
    FieldId field_id{};
    ArrXf buffer;
  };

  void checkFps(std::size_t control_freq);
  void initField(const yml::Node &node, FieldView &field_spec);

  std::string path_;
  std::unique_ptr<DataLoader> data_;
  std::size_t num_frames_{};
  std::vector<FieldView> fields_;

  std::size_t frame_idx_{};
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_MOTION_TRAJECTORY_H_

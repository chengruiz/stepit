#include <map>
#include <string>
#include <vector>

#include <stepit/policy_neuro/field.h>
#include <stepit/pyutils/npz_reader.h>

namespace stepit {
namespace neuro_policy {
class MotionTrajectory : public Module {
 public:
  MotionTrajectory(const PolicySpec &policy_spec, const std::string &home_dir);

  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &result) override;

 private:
  YAML::Node config_;
  std::string npz_filename_;
  NpzReader npz_;
  std::size_t num_frames_{};
  std::vector<std::string> key_names_;
  std::vector<std::string> field_names_;
  std::vector<std::size_t> field_sizes_;
  FieldIdVec field_ids_;

  std::size_t frame_idx_{};
};
}  // namespace neuro_policy
}  // namespace stepit

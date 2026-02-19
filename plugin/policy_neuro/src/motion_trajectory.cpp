#include <cmath>
#include <cstdint>
#include <functional>
#include <numeric>

#include <stepit/policy_neuro/motion_trajectory.h>

namespace stepit {
namespace neuro_policy {
MotionTrajectory::MotionTrajectory(const PolicySpec &policy_spec, const std::string &home_dir)
    : config_(yml::loadFile(home_dir + "/motion_trajectory.yml")) {
  npz_filename_ = yml::readIf<std::string>(config_["npz_filename"], "motion_trajectory.npz");
  npz_.readFile((npz_filename_[0] == '/') ? npz_filename_ : (home_dir + "/" + npz_filename_));

  if (npz_.hasKey("fps")) {
    const auto &fps = npz_["fps"];
    STEPIT_ASSERT(fps.shape.size() == 1 and fps.shape[0] == 1,
                  "Expected 'fps' in 'motion_trajectory.npz' to be a scalar.");
    std::size_t fps_value = 0;
    if (fps.dtype == "float64") {
      fps_value = static_cast<std::size_t>(std::round(*fps.data<double>()));
    } else if (fps.dtype == "float32") {
      fps_value = static_cast<std::size_t>(std::round(*fps.data<float>()));
    } else if (fps.dtype == "int32") {
      fps_value = static_cast<std::size_t>(*fps.data<int32_t>());
    } else if (fps.dtype == "int64") {
      fps_value = static_cast<std::size_t>(*fps.data<int64_t>());
    } else {
      STEPIT_ERROR("Expected 'fps' in '{}' to have dtype 'float32', 'float64', 'int32', or 'int64', but got '{}'.",
                   npz_filename_, fps.dtype);
    }
    STEPIT_ASSERT(fps_value == policy_spec.control_freq,
                  "FPS in '{}' ({}) does not match control frequency in policy spec ({}).", npz_filename_, fps_value,
                  policy_spec.control_freq);
  }

  STEPIT_ASSERT(config_["field"].IsDefined(), "Missing 'field' in motion_trajectory.yml.");
  STEPIT_ASSERT(config_["field"].IsSequence(), "Expected 'field' to be a sequence.");
  for (const auto &node : config_["field"]) {
    STEPIT_ASSERT(node.IsMap(), "Expected each field to be a map.");
    auto field_name = yml::readAs<std::string>(node, "name");
    auto key_name   = yml::readAs<std::string>(node, "key");
    STEPIT_ASSERT(npz_.hasKey(key_name), "Key '{}' not found in '{}'.", key_name, npz_filename_);

    const auto &array = npz_[key_name];
    const auto &shape = array.shape;
    STEPIT_ASSERT(shape.size() >= 2, "Expected array for key '{}' to have at least 2 dimensions.", key_name);
    STEPIT_ASSERT(array.dtype == "float32" or array.dtype == "float64",
                  "Expected array '{}' to have dtype 'float32' or 'float64', but got '{}'.", key_name, array.dtype);
    if (num_frames_ == 0) {
      num_frames_ = shape[0];
    } else if (num_frames_ != shape[0]) {  // raise error
      STEPIT_ERROR("Arrays in '{}' have different frame counts.", npz_filename_);
    }

    std::size_t field_size = std::accumulate(shape.begin() + 1, shape.end(), 1UL, std::multiplies<std::size_t>());
    if (yml::hasValue(node, "size")) {
      auto specified_size = yml::readAs<std::size_t>(node, "size");
      STEPIT_ASSERT(specified_size == field_size, "Field size specified ({}) does not match that ({}) of array '{}'.",
                    specified_size, field_size, key_name);
    }
    field_names_.push_back(field_name);
    key_names_.push_back(key_name);
    field_sizes_.push_back(field_size);
    field_ids_.push_back(registerProvision(field_name, static_cast<FieldSize>(field_size)));
  }

  STEPIT_ASSERT(num_frames_ > 0, "Loaded trajectory '{}' with 0 frames.", npz_filename_);
  STEPIT_DBUG("Loaded motion trajectory with {} frames and {} fields from '{}'.", num_frames_, field_names_.size(),
              npz_filename_);
}

bool MotionTrajectory::reset() {
  frame_idx_ = 0;
  return true;
}

bool MotionTrajectory::update(const LowState &low_state, ControlRequests &requests, FieldMap &context) {
  for (std::size_t i{}; i < field_ids_.size(); ++i) {
    const auto &key   = key_names_[i];
    const auto &array = npz_[key];

    const auto field_id   = field_ids_[i];
    const auto field_size = field_sizes_[i];

    std::size_t offset = frame_idx_ * field_size;
    if (array.dtype == "float64") {
      context[field_id] = cmArrXd(array.data<double>() + offset, field_size).cast<float>();
    } else {
      context[field_id] = cmArrXf(array.data<float>() + offset, field_size);
    }
  }

  if (frame_idx_ < num_frames_ - 1) ++frame_idx_;
  return true;
}

STEPIT_REGISTER_MODULE(motion_trajectory, kDefPriority, Module::make<MotionTrajectory>);
}  // namespace neuro_policy
}  // namespace stepit

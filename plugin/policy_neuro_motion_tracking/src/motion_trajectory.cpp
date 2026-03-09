#include <cmath>
#include <cstdint>
#include <functional>
#include <numeric>

#include <stepit/policy_neuro/motion_trajectory.h>

namespace stepit {
namespace neuro_policy {
MotionTrajectory::MotionTrajectory(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : Module(policy_spec, ModuleSpec(module_spec, "motion_trajectory")) {
  path_ = config_["path"].as<std::string>(name_);
  STEPIT_ASSERT(not path_.empty(), "'path' cannot be empty.");
  path_ = (path_[0] == '/') ? path_ : joinPaths(policy_spec.home_dir, path_);

  const auto dataloader_factory = config_["dataloader_factory"].as<std::string>("");
  data_                         = DataLoader::make(dataloader_factory, path_);

  const auto default_offsets = config_["offsets"].as<std::vector<std::int64_t>>({0});
  STEPIT_ASSERT(not default_offsets.empty(), "'offsets' cannot be empty.");

  checkFps(policy_spec.control_freq);

  const auto field_nodes = config_["field"];
  field_nodes.assertSequence();
  for (const auto &node : field_nodes) {
    FieldView field_spec;
    initField(node, field_spec);
    if (field_spec.offsets.empty()) field_spec.offsets = default_offsets;
    fields_.push_back(std::move(field_spec));
  }

  STEPIT_ASSERT(num_frames_ > 0, "Loaded trajectory '{}' with 0 frames.", path_);
  STEPIT_DBUG("Loaded trajectory with {} frames and {} fields from '{}'.", num_frames_, fields_.size(), path_);
}

void MotionTrajectory::checkFps(std::size_t control_freq) {
  if (data_->hasKey("fps")) {
    const auto &fps = (*data_)["fps"];
    STEPIT_ASSERT(product(fps.shape) == 1, "Expected 'fps' in '{}' to contain exactly one element.", path_);
    std::size_t fps_value = 0;
    if (fps.dtype == "float64") {
      fps_value = static_cast<std::size_t>(std::round(fps.at<double>(0)));
    } else if (fps.dtype == "float32") {
      fps_value = static_cast<std::size_t>(std::round(fps.at<float>(0)));
    } else if (fps.dtype == "int32") {
      fps_value = static_cast<std::size_t>(fps.at<int32_t>(0));
    } else if (fps.dtype == "int64") {
      fps_value = static_cast<std::size_t>(fps.at<int64_t>(0));
    } else {
      STEPIT_THROW("Expected 'fps' in '{}' to have dtype 'float32', 'float64', 'int32', or 'int64', but got '{}'.",
                   path_, fps.dtype);
    }
    STEPIT_ASSERT(fps_value == control_freq, "'fps' in '{}' ({}) does not match the control frequency ({}).", path_,
                  fps_value, control_freq);
  }
}

void MotionTrajectory::initField(const yml::Node &node, FieldView &field) {
  STEPIT_ASSERT(node.isMap(), "Expected each field to be a map.");
  const auto name = node["name"].as<std::string>();
  const auto key  = node["key"].as<std::string>();
  STEPIT_ASSERT(data_->hasKey(key), "Key '{}' not found in '{}'.", key, path_);
  const auto &source = (*data_)[key];
  const auto &shape  = source.shape;
  STEPIT_ASSERT(shape.size() >= 1, "Expected array for key '{}' to have at least 1 dimension.", key);
  STEPIT_ASSERT(source.dtype == "float32" or source.dtype == "float64",
                "Expected array '{}' to have dtype 'float32' or 'float64', but got '{}'.", key, source.dtype);

  if (num_frames_ == 0) {
    num_frames_ = shape[0];
  } else if (num_frames_ != shape[0]) {
    STEPIT_THROW("Arrays in '{}' have different frame counts.", path_);
  }

  std::size_t frame_size = std::accumulate(shape.begin() + 1, shape.end(), 1UL, std::multiplies<std::size_t>());
  std::vector<std::size_t> indices;
  if (node["indices"].hasValue()) {
    const auto indices_node = node["indices"];
    STEPIT_ASSERT(indices_node.isSequence() and indices_node.size() > 0,
                  "'indices' for field '{}' must be a non-empty sequence.", name);
    indices_node.to(indices);
    STEPIT_ASSERT(
        std::all_of(indices.begin(), indices.end(), [&frame_size](std::size_t index) { return index < frame_size; }),
        "'indices' of field '{}' should be in the range [0, {}).", name, frame_size);
  } else {
    std::size_t start = node["start"].as<std::size_t>(0UL);
    std::size_t end   = node["end"].as<std::size_t>(frame_size);
    STEPIT_ASSERT(start < frame_size, "Start index {} is out of range [0, {}) for field '{}' (key '{}').", start,
                  frame_size, name, key);
    STEPIT_ASSERT(end <= frame_size, "End index {} is out of range (0, {}] for field '{}' (key '{}').", end, frame_size,
                  name, key);
    STEPIT_ASSERT(end > start, "Slice range [start={}, end={}) for field '{}' is invalid.", start, end, name);
    for (auto index = start; index < end; ++index) {
      indices.push_back(static_cast<std::size_t>(index));
    }
  }

  node["offsets"].to(field.offsets, true);
  STEPIT_ASSERT(not field.offsets.empty(), "Offsets for field '{}' cannot be empty.", name);
  field.frame_size = indices.size();
  field.field_size = field.frame_size * field.offsets.size();
  if (node["size"].hasValue()) {
    const auto declared_size = node["size"].as<std::size_t>();
    STEPIT_ASSERT(declared_size == field.field_size,
                  "Field size specified ({}) does not match the stacked size ({}) of field '{}' (key '{}').",
                  declared_size, field.field_size, name, key);
  }

  field.source.resize(num_frames_);
  for (std::size_t frame{}; frame < num_frames_; ++frame) {
    const auto source_offset = frame * frame_size;
    auto &frame_data         = field.source[frame];
    frame_data.resize(static_cast<Eigen::Index>(field.frame_size));

    if (source.dtype == "float64") {
      for (std::size_t i{}; i < indices.size(); ++i) {
        frame_data[static_cast<Eigen::Index>(i)] = static_cast<float>(source.at<double>(source_offset + indices[i]));
      }
    } else {
      for (std::size_t i{}; i < indices.size(); ++i) {
        frame_data[static_cast<Eigen::Index>(i)] = source.at<float>(source_offset + indices[i]);
      }
    }
  }

  field.field_id = registerProvision(name, static_cast<FieldSize>(field.field_size));
  field.buffer.resize(static_cast<Eigen::Index>(field.field_size));
}

bool MotionTrajectory::reset() {
  frame_idx_ = 0;
  return true;
}

bool MotionTrajectory::update(const LowState &low_state, ControlRequests &requests, FieldMap &context) {
  const auto frame_idx     = static_cast<std::int64_t>(frame_idx_);
  const auto max_frame_idx = static_cast<std::int64_t>(num_frames_ - 1);
  for (auto &field : fields_) {
    const auto slice_size = static_cast<Eigen::Index>(field.frame_size);
    auto &buffer          = field.buffer;

    Eigen::Index write_offset = 0;
    for (const auto frame_offset : field.offsets) {
      buffer.segment(write_offset, slice_size) = field.source[static_cast<std::size_t>(
          clamp(frame_idx + frame_offset, std::int64_t{0}, max_frame_idx))];
      write_offset += slice_size;
    }
    context[field.field_id] = field.buffer;
  }

  if (frame_idx_ < num_frames_ - 1) ++frame_idx_;
  return true;
}

STEPIT_REGISTER_MODULE(motion_trajectory, kDefPriority, Module::make<MotionTrajectory>);
}  // namespace neuro_policy
}  // namespace stepit

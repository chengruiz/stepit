#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numeric>

#include <stepit/modular_policy/motion_clip.h>

namespace stepit {
namespace modular_policy {
MotionClip::MotionClip(std::string path, const std::string &dataloader_factory) : path_(std::move(path)) {
  STEPIT_ASSERT(not path_.empty(), "'path' cannot be empty.");
  data_ = field::DataLoader::make(dataloader_factory, path_);

  // Resolve fps if available
  if (data_->hasKey("fps")) {
    const auto &fps = (*data_)["fps"];
    STEPIT_ASSERT(product(fps.shape) == 1, "Expected 'fps' in '{}' to contain exactly one element.", path_);

    if (fps.dtype == "float64") {
      fps_ = fps.at<double>(0);
    } else if (fps.dtype == "float32") {
      fps_ = static_cast<double>(fps.at<float>(0));
    } else if (fps.dtype == "int32") {
      fps_ = static_cast<double>(fps.at<int32_t>(0));
    } else if (fps.dtype == "int64") {
      fps_ = static_cast<double>(fps.at<int64_t>(0));
    } else {
      STEPIT_THROW("Expected 'fps' in '{}' to have dtype 'float32', 'float64', 'int32', or 'int64', but got '{}'.",
                   path_, fps.dtype);
    }

    STEPIT_ASSERT(fps_ > 0., "'fps' should be positive, but got {} in '{}'.", fps_, path_);
  }
}

std::vector<ArrXf> MotionClip::loadField(const std::string &name, const std::string &key, yml::Indices indices,
                                         const std::string &type, bool differentiate, double output_fps) {
  // Load the field properties and validate the source data
  const auto data_type = parseDataType(type);
  STEPIT_ASSERT(data_->hasKey(key), "Key '{}' not found in '{}'.", key, path_);

  const auto &source = (*data_)[key];
  const auto &shape  = source.shape;
  STEPIT_ASSERT(shape.size() >= 1, "Expected array for key '{}' to have at least 1 dimension.", key);
  STEPIT_ASSERT(source.dtype == "float32" or source.dtype == "float64",
                "Expected array '{}' to have dtype 'float32' or 'float64', but got '{}'.", key, source.dtype);

  const auto source_num_frames = shape[0];
  STEPIT_ASSERT(source_num_frames > 0, "Expected array for key '{}' to have at least one frame, but got 0.", key);
  const auto source_frame_size   = std::accumulate(shape.begin() + 1, shape.end(), std::size_t{1},
                                                   std::multiplies<std::size_t>());
  const auto canonical_indices   = indices.canonicalize(source_frame_size);
  const auto selected_frame_size = canonical_indices.size();
  if (data_type == DataType::kQuaternion) {
    STEPIT_ASSERT(selected_frame_size % 4 == 0,
                  "Field '{}' (key '{}') requires a per-frame size divisible by 4, but got {}.", name, key,
                  selected_frame_size);
  }

  // Extract the selected indices into contiguous frames
  std::vector<ArrXf> source_frames(source_num_frames);
  for (std::size_t frame_index{}; frame_index < source_num_frames; ++frame_index) {
    const auto source_offset = frame_index * source_frame_size;
    auto &frame_data         = source_frames[frame_index];
    frame_data.resize(static_cast<Eigen::Index>(indices.size()));

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

  // Resample and optionally differentiate the frames
  auto sampled_source = resampleFrames(source_frames, output_fps, data_type);
  return differentiate ? differentiateFrames(sampled_source, output_fps, data_type) : sampled_source;
}

auto MotionClip::parseDataType(const std::string &type) -> DataType {
  if (type.empty() or type == "numeric") return DataType::kNumeric;
  if (type == "quaternion") return DataType::kQuaternion;
  STEPIT_THROW("Unsupported field type '{}'. Expected 'numeric' or 'quaternion'.", type);
}

ArrXf MotionClip::interpolateFrame(const std::vector<ArrXf> &source, DataType data_type, double source_pos) const {
  const auto index0 = static_cast<std::size_t>(std::floor(source_pos));
  const auto index1 = std::min(index0 + 1, source.size() - 1);
  if (index0 == index1) return source[index0];

  const auto &start = source[index0];
  const auto &end   = source[index1];
  const auto blend  = static_cast<float>(source_pos - static_cast<double>(index0));
  if (data_type == DataType::kNumeric) {
    return start * (1.F - blend) + end * blend;
  } else if (data_type == DataType::kQuaternion) {
    ArrXf out(start.size());
    for (Eigen::Index offset{}; offset < start.size(); offset += 4) {
      Quatf q0(start.segment(offset, 4));
      Quatf q1(end.segment(offset, 4));
      out.segment(offset, 4) = q0.slerp(q1, blend).coeffs();
    }
    return out;
  } else {
    STEPIT_UNREACHABLE();
  }
}

std::vector<ArrXf> MotionClip::resampleFrames(const std::vector<ArrXf> &source, double output_fps,
                                              DataType data_type) const {
  const auto source_fps = fps_ > 0. ? fps_ : output_fps;
  if (source.empty() or isApprox(source_fps, output_fps)) return source;
  const auto source_size = static_cast<double>(source.size() - 1);
  const double ratio     = source_fps / output_fps;

  const std::size_t output_frame_count = 1UL + static_cast<std::size_t>(std::round(source_size / ratio));
  std::vector<ArrXf> sampled_source(output_frame_count);
  for (std::size_t frame{}; frame < output_frame_count; ++frame) {
    const auto source_pos = clamp(static_cast<double>(frame) * ratio, 0., source_size);
    sampled_source[frame] = interpolateFrame(source, data_type, source_pos);
  }
  return sampled_source;
}

std::vector<ArrXf> MotionClip::differentiateFrames(const std::vector<ArrXf> &source, double fps,
                                                   DataType data_type) const {
  if (source.empty()) return {};
  std::size_t output_frame_size{};
  if (data_type == DataType::kNumeric) {
    output_frame_size = static_cast<std::size_t>(source.front().size());
  } else if (data_type == DataType::kQuaternion) {
    output_frame_size = 3 * (static_cast<std::size_t>(source.front().size()) / 4);
  } else {
    STEPIT_UNREACHABLE();
  }
  if (source.size() == 1) return {ArrXf::Zero(static_cast<Eigen::Index>(output_frame_size))};

  std::vector<ArrXf> differentiated(source.size());
  for (std::size_t frame_index{}; frame_index < source.size(); ++frame_index) {
    std::size_t prev_frame_index = frame_index == 0 ? 0 : frame_index - 1;
    std::size_t next_frame_index = (frame_index + 1 < source.size()) ? frame_index + 1 : source.size() - 1;
    const auto &prev_frame       = source[prev_frame_index];
    const auto &next_frame       = source[next_frame_index];

    auto &out = differentiated[frame_index];
    float dt  = static_cast<float>(next_frame_index - prev_frame_index) / static_cast<float>(fps);

    if (data_type == DataType::kNumeric) {
      out = (next_frame - prev_frame) / dt;
    } else if (data_type == DataType::kQuaternion) {
      out.resize(static_cast<Eigen::Index>(output_frame_size));
      for (Eigen::Index offset{}; offset < prev_frame.size(); offset += 4) {
        const auto vec_offset = 3 * (offset / 4);
        Quatf q_prev(prev_frame.segment(offset, 4));
        Quatf q_next(next_frame.segment(offset, 4));
        out.segment(vec_offset, 3) = (q_next * q_prev.inverse()).rotationVector().array() / dt;
      }
    } else {
      STEPIT_UNREACHABLE();
    }
  }
  return differentiated;
}
}  // namespace modular_policy
}  // namespace stepit

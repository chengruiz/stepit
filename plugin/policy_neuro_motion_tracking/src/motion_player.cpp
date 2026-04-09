#include <cstddef>
#include <cstdint>
#include <set>

#include <stepit/policy_neuro/motion_player.h>

namespace stepit {
namespace neuro_policy {
// clang-format off
const std::map<std::string, MotionPlayer::Action> MotionPlayer::kActionMap = {
    {"ReplayCurrentClip",  Action::kReplayCurrentClip},
    {"SelectNextClip",     Action::kSelectNextClip},
    {"SelectPreviousClip", Action::kSelectPreviousClip},
};
// clang-format on

MotionPlayer::MotionPlayer(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : Module(policy_spec, ModuleSpec(module_spec, "motion_player")) {
  // Load field specifications
  const auto default_offsets = config_["offsets"].as<std::vector<std::int64_t>>({0});
  STEPIT_ASSERT(not default_offsets.empty(), "'offsets' cannot be empty.");
  const auto field_nodes = config_["field"];
  field_nodes.assertNonEmptySequence();
  std::set<std::string> field_names;
  for (const auto &node : field_nodes) {
    FieldSpec field_spec;
    node["name"].to(field_spec.name);
    if (not field_names.insert(field_spec.name).second) {
      node.throwError(fmt::format("Duplicate field name '{}'", field_spec.name));
    }
    node["key"].to(field_spec.key);
    field_spec.type          = node["type"].as<std::string>("numeric");
    field_spec.differentiate = node["differentiate"].as<bool>(false);
    field_spec.indices       = node.as<yml::Indices>();
    field_spec.field_size    = node["size"].as<std::size_t>(0);
    node["offsets"].to(field_spec.offsets, true);
    if (field_spec.offsets.empty()) field_spec.offsets = default_offsets;
    field_specs_.push_back(std::move(field_spec));
  }

  // Load motion clip paths and dataloader
  auto path_node = config_["path"];
  std::vector<std::string> motion_paths;
  if (not path_node.hasValue()) {
    motion_paths.push_back("motion");
  } else if (path_node.isScalar()) {
    motion_paths.push_back(path_node.as<std::string>());
  } else if (path_node.isSequence()) {
    path_node.throwIf(path_node.size() == 0, "'path' sequence cannot be empty");
    path_node.to(motion_paths);
  } else {
    path_node.throwError("'path' should be a string or a list of strings");
  }
  for (auto &motion_path : motion_paths) {
    path_node.throwIf(motion_path.empty(), "'path' must not contain empty strings");
    if (motion_path[0] != '/') motion_path = joinPaths(policy_spec.home_dir, motion_path);
  }

  // Load motion clips and fields, and check for consistency
  const auto target_freq = static_cast<double>(policy_spec.control_freq);
  STEPIT_ASSERT(target_freq > 0., "Control frequency should be positive, but got {}.", target_freq);
  const auto dataloader_factory = config_["dataloader_factory"].as<std::string>("");
  for (const auto &motion_path : motion_paths) {
    MotionData motion;
    motion.name = fs::path(motion_path).stem().string();

    // Load fields for the motion clip according to the field specifications
    MotionClip motion_clip(motion_path, dataloader_factory);
    for (auto &field_spec : field_specs_) {
      auto field_frames = motion_clip.loadField(field_spec.name, field_spec.key, field_spec.indices, field_spec.type,
                                                field_spec.differentiate, target_freq);

      // Check that all fields have the same number of frames
      const auto num_frames = field_frames.size();
      if (num_frames == 0) {
        STEPIT_THROW("Field '{}' (key '{}') in '{}' has 0 frames.", field_spec.name, field_spec.key,
                     motion_clip.path());
      } else if (motion.num_frames == 0) {
        motion.num_frames = num_frames;
      } else if (num_frames != motion.num_frames) {
        STEPIT_THROW("Field '{}' (key '{}') resolves to {} control frames, but previous fields resolved to {}.",
                     field_spec.name, field_spec.key, num_frames, motion.num_frames);
      }

      // Initialize sizes on the first loaded clip and check for consistency on subsequent clips
      if (field_spec.frame_size == 0) {
        field_spec.frame_size = static_cast<std::size_t>(field_frames.front().size());
      } else if (field_spec.frame_size != static_cast<std::size_t>(field_frames.front().size())) {
        STEPIT_THROW("Inconsistent frame size for field '{}' (key '{}').", field_spec.name, field_spec.key);
      }
      if (field_spec.field_size == 0) {
        field_spec.field_size = field_spec.frame_size * field_spec.offsets.size();
      } else if (field_spec.field_size != field_spec.frame_size * field_spec.offsets.size()) {
        STEPIT_THROW("Inconsistent field size for field '{}' (key '{}').", field_spec.name, field_spec.key);
      }
      motion.fields.push_back(field_frames);
    }
    STEPIT_DBUG("Loaded motion with {} frames from '{}'.", motion.num_frames, motion_clip.path());
    motions_.push_back(std::move(motion));
  }

  // Register the provision for the field
  for (auto &field_spec : field_specs_) {
    field_spec.field_id = registerProvision(field_spec.name, static_cast<FieldSize>(field_spec.field_size));
  }
  motion_frame_index_id_ = registerProvision("motion_frame_index", 1);
  // Initialize buffers for each field
  buffers_.resize(field_specs_.size());
  for (std::size_t i{}; i < field_specs_.size(); ++i) {
    buffers_[i].resize(static_cast<Eigen::Index>(field_specs_[i].field_size));
  }
}

bool MotionPlayer::reset() {
  clip_index_  = 0;
  frame_index_ = 0;

  joystick_rules_.emplace_back([](const joystick::State &js) -> std::string {
    if (js.Select().on_press) return "Policy/Motion/ReplayCurrentClip";
    if (js.LB().pressed and js.Start().on_press) return "Policy/Motion/SelectPreviousClip";
    if (js.RB().pressed and js.Start().on_press) return "Policy/Motion/SelectNextClip";
    return "";
  });
  return true;
}

bool MotionPlayer::update(const LowState &, ControlRequests &requests, FieldMap &context) {
  for (auto &&request : requests.filterByChannel("Policy/Motion")) {
    handleControlRequest(std::move(request));
  }

  // Write the current frame of the current clip to the context according to the field specifications
  const auto &motion = motions_[clip_index_];
  for (std::size_t field_index{}; field_index < field_specs_.size(); ++field_index) {
    const auto &field_spec = field_specs_[field_index];
    const auto &field      = motion.fields[field_index];
    const auto frame_size  = static_cast<Eigen::Index>(field_spec.frame_size);
    auto &buffer           = buffers_[field_index];

    Eigen::Index write_offset = 0;
    for (const auto frame_offset : field_spec.offsets) {
      const std::int64_t sample_index = clamp(static_cast<std::int64_t>(frame_index_) + frame_offset, std::int64_t{0},
                                              static_cast<std::int64_t>(motions_[clip_index_].num_frames - 1));
      buffer.segment(write_offset, frame_size) = field[static_cast<std::size_t>(sample_index)];
      write_offset += frame_size;
    }
    context[field_spec.field_id] = buffer;
  }
  context[motion_frame_index_id_] = ArrXf::Constant(1, static_cast<float>(frame_index_));

  if (frame_index_ + 1 < motions_[clip_index_].num_frames) ++frame_index_;
  return true;
}

void MotionPlayer::handleControlRequest(ControlRequest request) {
  auto action = lookupAction(request.action(), kActionMap);
  switch (action) {
    case Action::kReplayCurrentClip: {
      frame_index_ = 0;
      request.response(kSuccess);
      STEPIT_LOG("Replaying motion clip '{}'.", motions_[clip_index_].name);
      break;
    }
    case Action::kSelectNextClip: {
      clip_index_  = (clip_index_ + 1) % motions_.size();
      frame_index_ = 0;
      STEPIT_LOG("Switched to motion clip '{}'.", motions_[clip_index_].name);
      request.response(kSuccess);
      break;
    }
    case Action::kSelectPreviousClip: {
      clip_index_  = (clip_index_ + motions_.size() - 1) % motions_.size();
      frame_index_ = 0;
      STEPIT_LOG("Switched to motion clip '{}'.", motions_[clip_index_].name);
      request.response(kSuccess);
      break;
    }
    default: {
      request.response(kUnrecognizedRequest);
      break;
    }
  }
}

void MotionPlayer::exit() { joystick_rules_.clear(); }

STEPIT_REGISTER_MODULE(motion_player, kDefPriority, Module::make<MotionPlayer>);
}  // namespace neuro_policy
}  // namespace stepit

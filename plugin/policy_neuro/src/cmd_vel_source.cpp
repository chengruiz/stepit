#include <cmath>

#include <stepit/agent.h>
#include <stepit/policy_neuro/cmd_vel_source.h>

namespace stepit {
namespace neuro_policy {
constexpr std::array<const char *, CmdVelSource::kNumModes> CmdVelSource::kModeName;

// clang-format off
const std::map<std::string, CmdVelSource::Action> CmdVelSource::kActionMap = {
    {"SetVelocity",         Action::kSetVelocity},
    {"SetVelocityUnscaled", Action::kSetVelocityUnscaled},
    {"SetTurboRatio",       Action::kSetTurboRatio},
    {"SelectMode",          Action::kSelectMode},
    {"CycleMode",           Action::kCycleMode},
    {"EnableSmoothing",     Action::kEnableSmoothing},
    {"DisableSmoothing",    Action::kDisableSmoothing},
    {"SetMaxAccel",         Action::kSetMaxAccel},
    {"EnableJoystick",      Action::kEnableJoystick},
    {"DisableJoystick",     Action::kDisableJoystick},
};
// clang-format on

CmdVelSource::CmdVelSource(const PolicySpec &policy_spec, const std::string &home_dir) {
  cmd_vel_id_   = registerProvision("cmd_vel", 3);
  cmd_stall_id_ = registerProvision("cmd_stall", 1);
  timestep_     = 1.0F / static_cast<float>(policy_spec.control_freq);

  if (fs::exists(home_dir + "/cmd_vel.yml")) {
    config_ = yml::loadFile(home_dir + "/cmd_vel.yml");
    yml::setIf(config_, "velocity_scale_factor", velocity_scale_factor_);
    yml::setIf(config_, "velocity_turbo_factor", velocity_turbo_factor_);
    yml::setIf(config_, "velocity_deadzone", velocity_deadzone_);
    yml::setIf(config_, "smoothing", smoothing_);
    yml::setIf(config_, "max_acceleration", max_acceleration_);
    yml::setIf(config_, "stall_mode_enabled", mode_enabled_[kStall]);
    yml::setIf(config_, "move_mode_enabled", mode_enabled_[kMove]);
    yml::setIf(config_, "joystick_enabled", joystick_enabled_);
  }
}

bool CmdVelSource::reset() {
  mode_ = kAuto;
  cmd_vel_.setZero();
  cmd_stall_ = true;

  joystick_rules_.emplace_back([this](const joystick::State &js) -> std::string {
    if (not joystick_enabled_) return "";
    return fmt::format("Policy/CmdVel/SetTurboRatio:{}", (js.rt() + 1.0) / 2.0);
  });
  joystick_rules_.emplace_back([this](const joystick::State &js) -> std::string {
    if (not joystick_enabled_) return "";
    return fmt::format("Policy/CmdVel/SetVelocityUnscaled:{},{},{}", -js.las_y(), -js.las_x(), -js.ras_x());
  });
  joystick_rules_.emplace_back([](const joystick::State &js) -> std::string {
    return js.Start().on_press ? "Policy/CmdVel/CycleMode" : "";
  });
  return true;
}

bool CmdVelSource::update(const LowState &low_state, ControlRequests &requests, FieldMap &result) {
  for (auto &&request : requests.filterByChannel("Policy/CmdVel")) {
    handleControlRequest(std::move(request));
  }

  if (smoothing_) {
    cmd_vel_ += (target_cmd_vel_ - cmd_vel_)
                    .cwiseMin(max_acceleration_ * timestep_)
                    .cwiseMax(-max_acceleration_ * timestep_);
  } else {
    cmd_vel_ = target_cmd_vel_;
  }

  switch (mode_) {
    case kAuto:
      cmd_stall_ = cmd_vel_.abs().maxCoeff() < velocity_deadzone_ and target_cmd_vel_.abs()
                                                                              .maxCoeff() < velocity_deadzone_;
      if (cmd_stall_) cmd_vel_.setZero();
      break;
    case kStall:
      cmd_stall_ = true;
      cmd_vel_.setZero();
      break;
    case kMove:
      cmd_stall_ = false;
      break;
    default:
      STEPIT_ERROR("Unintended condition!");
      break;
  }

  result[cmd_vel_id_]   = cmd_vel_;
  result[cmd_stall_id_] = Arr1f{cmd_stall_};
  return true;
}

void CmdVelSource::exit() { joystick_rules_.clear(); }

void CmdVelSource::handleControlRequest(ControlRequest request) {
  auto action = lookupAction(request.action(), kActionMap);
  switch (action) {
    case Action::kSetVelocity:
    case Action::kSetVelocityUnscaled: {
      float vx, vy, wz;
      if (not request.parseArgument("%f,%f,%f", vx, vy, wz) or
          (not std::isfinite(vx) or not std::isfinite(vy) or not std::isfinite(wz))) {
        request.response(kIncorrectArgument);
        break;
      }
      if (action == Action::kSetVelocity) {
        target_cmd_vel_ = Arr3f{vx, vy, wz};
      } else {
        Vec3f unscaled_cmd_vel{vx, vy, wz};
        unscaled_cmd_vel = unscaled_cmd_vel / std::max(1.0F, unscaled_cmd_vel.norm());  // clamp norm to 1.0
        target_cmd_vel_  = unscaled_cmd_vel.array()
                              .cwiseProduct(velocity_scale_factor_)
                              .cwiseProduct(1 + velocity_turbo_factor_ * velocity_turbo_ratio_);
      }
      request.response(kSuccess);
      break;
    }
    case Action::kSetTurboRatio: {
      float turbo_ratio;
      if (not request.parseArgument("%f", turbo_ratio) or
          (not std::isfinite(turbo_ratio) or turbo_ratio < -kEPS or turbo_ratio > 1.0F + kEPS)) {
        request.response(kIncorrectArgument);
        break;
      }
      turbo_ratio = clamp(turbo_ratio, 0.0F, 1.0F);
      // Apply new turbo ratio
      target_cmd_vel_ = target_cmd_vel_.cwiseQuotient(1 + velocity_turbo_factor_ * velocity_turbo_ratio_)
                            .cwiseProduct(1 + velocity_turbo_factor_ * turbo_ratio);
      velocity_turbo_ratio_ = turbo_ratio;
      request.response(kSuccess);
      break;
    }
    case Action::kSelectMode: {
      auto it = std::find_if(kModeName.begin(), kModeName.end(),
                             [&request](const char *mode) { return request.argument() == mode; });
      if (it == kModeName.end()) {
        request.response(kIncorrectArgument, fmt::format("Unknown mode: '{}'.", request.argument()));
      } else {
        mode_ = static_cast<Mode>(std::distance(kModeName.begin(), it));
        request.response(kSuccess, fmt::format("Switched to '{}' mode.", kModeName[mode_]));
      }
      break;
    }
    case Action::kCycleMode: {
      do {
        mode_ = static_cast<Mode>((static_cast<int>(mode_) + 1) % kNumModes);
      } while (not mode_enabled_[mode_]);
      request.response(kSuccess, fmt::format("Switched to '{}' mode.", kModeName[mode_]));
      break;
    }
    case Action::kEnableSmoothing: {
      smoothing_ = true;
      request.response(kSuccess);
      break;
    }
    case Action::kDisableSmoothing: {
      smoothing_ = false;
      request.response(kSuccess);
      break;
    }
    case Action::kSetMaxAccel: {
      float ax, ay, az;
      if (not request.parseArgument("%f,%f,%f", ax, ay, az) or
          (not std::isfinite(ax) or not std::isfinite(ay) or not std::isfinite(az)) or (ax < 0 or ay < 0 or az < 0)) {
        request.response(kIncorrectArgument);
        break;
      }
      max_acceleration_ = Arr3f{ax, ay, az};
      request.response(kSuccess);
      break;
    }
    case Action::kEnableJoystick: {
      joystick_enabled_ = true;
      request.response(kSuccess);
      break;
    }
    case Action::kDisableJoystick: {
      joystick_enabled_ = false;
      request.response(kSuccess);
      break;
    }
    default: {
      request.response(kUnrecognizedRequest);
      break;
    }
  }
}

STEPIT_REGISTER_MODULE(cmd_vel, kDefPriority - 1, Module::make<CmdVelSource>);
STEPIT_REGISTER_FIELD_SOURCE(cmd_vel, kDefPriority - 1, Module::make<CmdVelSource>);
STEPIT_REGISTER_FIELD_SOURCE(cmd_stall, kDefPriority - 1, Module::make<CmdVelSource>);
}  // namespace neuro_policy
}  // namespace stepit

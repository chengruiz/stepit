#include <stepit/policy_neuro/cmd_posture_source.h>

namespace stepit {
namespace neuro_policy {
// clang-format off
const std::map<std::string, CmdRollSource::Action> CmdRollSource::kActionMap = {
    {"SetRoll",         Action::kSetRoll},
    {"SetRollUnscaled", Action::kSetRollUnscaled},
    {"EnableJoystick",  Action::kEnableJoystick},
    {"DisableJoystick", Action::kDisableJoystick},
};
// clang-format on

CmdRollSource::CmdRollSource(const PolicySpec &policy_spec, const std::string &home_dir) {
  cmd_roll_id_ = registerProvision("cmd_roll", 1);

  if (fs::exists(home_dir + "/cmd_posture.yml")) {
    config_ = yml::loadFile(home_dir + "/cmd_posture.yml");
    yml::setIf(config_, "roll_scale_factor", roll_scale_factor_);
    yml::setIf(config_, "joystick_enabled", joystick_enabled_);
  }
}

bool CmdRollSource::reset() {
  cmd_roll_ = 0.0F;
  joystick_rules_.emplace_back([this](const joystick::State &js) -> std::string {
    if (not joystick_enabled_) return "";
    float cmd_roll = static_cast<float>(js.Right().pressed) - static_cast<float>(js.Left().pressed);
    return fmt::format("Policy/CmdRoll/SetRollUnscaled:{}", cmd_roll);
  });
  return true;
}

bool CmdRollSource::update(const LowState &low_state, ControlRequests &requests, FieldMap &result) {
  for (auto &&request : requests.filterByChannel("Policy/CmdRoll")) {
    handleControlRequest(std::move(request));
  }

  result[cmd_roll_id_] = Arr1f{cmd_roll_};
  return true;
}

void CmdRollSource::exit() { joystick_rules_.clear(); }

void CmdRollSource::handleControlRequest(ControlRequest request) {
  auto action = lookupAction(request.action(), kActionMap);
  switch (action) {
    case Action::kSetRoll:
    case Action::kSetRollUnscaled: {
      float roll;
      if (not request.parseArgument("%f", roll) or not std::isfinite(roll)) {
        request.response(kIncorrectArgument);
        break;
      }
      if (action == Action::kSetRollUnscaled) roll *= roll_scale_factor_;
      cmd_roll_ = clamp(roll, -roll_scale_factor_, roll_scale_factor_);
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

// clang-format off
const std::map<std::string, CmdPitchSource::Action> CmdPitchSource::kActionMap = {
    {"SetPitch",         Action::kSetPitch},
    {"SetPitchUnscaled", Action::kSetPitchUnscaled},
    {"EnableJoystick",   Action::kEnableJoystick},
    {"DisableJoystick",  Action::kDisableJoystick},
};
// clang-format on

CmdPitchSource::CmdPitchSource(const PolicySpec &policy_spec, const std::string &home_dir) {
  cmd_pitch_id_ = registerProvision("cmd_pitch", 1);

  if (fs::exists(home_dir + "/cmd_posture.yml")) {
    config_ = yml::loadFile(home_dir + "/cmd_posture.yml");
    yml::setIf(config_, "pitch_scale_factor", pitch_scale_factor_);
    yml::setIf(config_, "joystick_enabled", joystick_enabled_);
  }
}

bool CmdPitchSource::reset() {
  cmd_pitch_ = 0.0F;
  joystick_rules_.emplace_back([this](const joystick::State &js) -> std::string {
    if (not joystick_enabled_) return "";
    return fmt::format("Policy/CmdPitch/SetPitchUnscaled:{}", js.ras_y());
  });
  return true;
}

bool CmdPitchSource::update(const LowState &low_state, ControlRequests &requests, FieldMap &result) {
  for (auto &&request : requests.filterByChannel("Policy/CmdPitch")) {
    handleControlRequest(std::move(request));
  }

  result[cmd_pitch_id_] = Arr1f{cmd_pitch_};
  return true;
}

void CmdPitchSource::exit() { joystick_rules_.clear(); }

void CmdPitchSource::handleControlRequest(ControlRequest request) {
  auto action = lookupAction(request.action(), kActionMap);
  switch (action) {
    case Action::kSetPitch:
    case Action::kSetPitchUnscaled: {
      float pitch;
      if (not request.parseArgument("%f", pitch) or not std::isfinite(pitch)) {
        request.response(kIncorrectArgument);
        break;
      }
      if (action == Action::kSetPitchUnscaled) pitch *= pitch_scale_factor_;
      cmd_pitch_ = clamp(pitch, -pitch_scale_factor_, pitch_scale_factor_);
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

// clang-format off
const std::map<std::string, CmdHeightSource::Action> CmdHeightSource::kActionMap = {
    {"SetHeight",       Action::kSetHeight},
    {"IncreaseHeight",  Action::kIncreaseHeight},
    {"DecreaseHeight",  Action::kDecreaseHeight},
    {"EnableJoystick",  Action::kEnableJoystick},
    {"DisableJoystick", Action::kDisableJoystick},
};
// clang-format on

CmdHeightSource::CmdHeightSource(const PolicySpec &policy_spec, const std::string &home_dir) {
  cmd_height_id_ = registerProvision("cmd_height", 1);

  if (fs::exists(home_dir + "/cmd_posture.yml")) {
    config_ = yml::loadFile(home_dir + "/cmd_posture.yml");
    yml::setIf(config_, "default_cmd_height", default_cmd_height_);
    yml::setIf(config_, "height_scale_factor", height_scale_factor_);
    yml::setIf(config_, "height_range", height_range_);
    yml::setIf(config_, "joystick_enabled", joystick_enabled_);
  }
}

bool CmdHeightSource::reset() {
  cmd_height_ = default_cmd_height_;
  joystick_rules_.emplace_back([this](const joystick::State &js) -> std::string {
    return (joystick_enabled_ and js.Up().on_press) ? "Policy/CmdHeight/IncreaseHeight" : "";
  });
  joystick_rules_.emplace_back([this](const joystick::State &js) -> std::string {
    return (joystick_enabled_ and js.Down().on_press) ? "Policy/CmdHeight/DecreaseHeight" : "";
  });
  return true;
}

bool CmdHeightSource::update(const LowState &low_state, ControlRequests &requests, FieldMap &result) {
  for (auto &&request : requests.filterByChannel("Policy/CmdHeight")) {
    handleControlRequest(std::move(request));
  }

  result[cmd_height_id_] = Arr1f{cmd_height_};
  return true;
}

void CmdHeightSource::exit() { joystick_rules_.clear(); }

void CmdHeightSource::handleControlRequest(ControlRequest request) {
  switch (lookupAction(request.action(), kActionMap)) {
    case Action::kSetHeight: {
      float height;
      if (not request.parseArgument("%f", height) or not std::isfinite(height)) {
        request.response(kIncorrectArgument);
        break;
      }
      cmd_height_ = clamp(height, height_range_.lower(), height_range_.upper());
      request.response(kSuccess);
      break;
    }
    case Action::kIncreaseHeight: {
      cmd_height_ = clamp(cmd_height_ + height_scale_factor_, height_range_.lower(), height_range_.upper());
      request.response(kSuccess);
      break;
    }
    case Action::kDecreaseHeight: {
      cmd_height_ = clamp(cmd_height_ - height_scale_factor_, height_range_.lower(), height_range_.upper());
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

STEPIT_REGISTER_MODULE(cmd_roll, kDefPriority - 1, Module::make<CmdRollSource>);
STEPIT_REGISTER_MODULE(cmd_pitch, kDefPriority - 1, Module::make<CmdPitchSource>);
STEPIT_REGISTER_MODULE(cmd_height, kDefPriority - 1, Module::make<CmdHeightSource>);
STEPIT_REGISTER_FIELD_SOURCE(cmd_roll, kDefPriority - 1, Module::make<CmdRollSource>);
STEPIT_REGISTER_FIELD_SOURCE(cmd_pitch, kDefPriority - 1, Module::make<CmdPitchSource>);
STEPIT_REGISTER_FIELD_SOURCE(cmd_height, kDefPriority - 1, Module::make<CmdHeightSource>);
}  // namespace neuro_policy
}  // namespace stepit

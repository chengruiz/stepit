#include <cmath>

#include <stepit/robot.h>

namespace stepit {
void OrientationLimit::load(const yml::Node &config) {
  if (not config.hasValue()) return;
  config["enabled"].to(enabled, true);
  config["roll"].to(roll, true);
  config["pitch"].to(pitch, true);
}

bool OrientationLimit::isViolated(const LowState &state) const {
  return enabled and (std::abs(state.imu.rpy[0]) > roll or std::abs(state.imu.rpy[1]) > pitch);
}

void NegativeJointPowerLimit::load(const yml::Node &config, std::size_t dof) {
  if (not config.hasValue()) return;

  config["enabled"].to(enabled, true);
  if (config["limits"].hasValue()) {
    limits.resize(dof);
    config["limits"].to(limits);
  }

  if (not enabled) return;
  STEPIT_ASSERT_EQ(limits.size(), dof, "Negative joint power limit config 'limits' size mismatch.");
  for (std::size_t i{}; i < dof; ++i) {
    const float limit = limits[i];
    STEPIT_ASSERT(std::isfinite(limit) and limit >= 0.0F,
                  "Negative joint power limit config 'limits' must contain finite, non-negative values "
                  "(invalid value at index {}).",
                  i);
  }
}

bool NegativeJointPowerLimit::apply(const LowState &state, const LowCmd &requested_cmd, LowCmd &filtered_cmd) const {
  STEPIT_ASSERT_EQ(state.motor_state.size(), requested_cmd.size(), "Negative-joint-power state size mismatch.");

  filtered_cmd = requested_cmd;
  if (not enabled) return false;
  STEPIT_ASSERT_EQ(limits.size(), requested_cmd.size(), "Negative-joint-power limit size mismatch.");

  bool applied = false;
  for (std::size_t i{}; i < requested_cmd.size(); ++i) {
    const auto &motor_state = state.motor_state[i];
    const auto &requested   = requested_cmd[i];
    const float limit       = limits[i];

    const float modeled_torque = requested.tor + requested.Kp * (requested.q - motor_state.q) +
                                 requested.Kd * (requested.dq - motor_state.dq);
    const float modeled_power = modeled_torque * motor_state.dq;
    if (modeled_power >= -limit) continue;

    applied                    = true;
    const float safe_torque    = -limit / motor_state.dq;
    float remaining_correction = safe_torque - modeled_torque;
    auto &filtered             = filtered_cmd[i];

    const bool feedforward_can_help = (requested.tor < 0.0F and remaining_correction > 0.0F) or
                                      (requested.tor > 0.0F and remaining_correction < 0.0F);
    if (feedforward_can_help) {
      const float feedforward_correction = std::copysign(
          std::min(std::abs(requested.tor), std::abs(remaining_correction)), remaining_correction);
      filtered.tor += feedforward_correction;
      remaining_correction -= feedforward_correction;
    }

    if (remaining_correction != 0.0F) {
      if (std::abs(requested.Kp) > 1e-3) {
        filtered.q += remaining_correction / requested.Kp;
      } else {
        filtered.tor += remaining_correction;
      }
    }
  }
  return applied;
}

void Safety::load(const yml::Node &config, std::size_t dof) {
  if (not config.hasValue()) return;
  orientation_limit.load(config["orientation_limit"]);
  negative_joint_power_limit.load(config["negative_joint_power_limit"], dof);
}
}  // namespace stepit

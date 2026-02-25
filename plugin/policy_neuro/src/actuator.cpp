#include <stepit/policy_neuro/actuator.h>

namespace stepit {
namespace neuro_policy {
Actuator::Actuator(const NeuroPolicySpec &policy_spec, const std::string &name)
    : Module(policy_spec, nonEmptyOr(name, "actuator"), true) {
  YAML::Node policy_config = yml::loadFile(joinPaths(policy_spec.home_dir, "policy.yml"));
  yml::assertHasValue(policy_config, "actuator");
  config_ = policy_config["actuator"];
  STEPIT_ASSERT(config_.IsMap(), "'actuator' entry in policy.yml should be a map.");

  scale_.setOnes(policy_spec.dof);
  bias_.setZero(policy_spec.dof);
  kp_.setZero(policy_spec.dof);
  kd_.setZero(policy_spec.dof);

  if (yml::hasValue(config_, "parameters")) {
    yml::assertSequenceOfSize(config_, "parameters", policy_spec.dof);
    yml::Node parameters = config_["parameters"];
    for (std::size_t i{}; i < policy_spec.dof; ++i) {
      STEPIT_ASSERT(parameters[i].IsMap(), "Each element in 'parameters' should be a map.");
      auto param = parameters[i];
      yml::setIf(param, "scale", scale_[i]);
      yml::setIf(param, "bias", bias_[i]);
      yml::setIf(param, "Kp", kp_[i]);
      yml::setIf(param, "Kd", kd_[i]);
    }
  } else {
    yml::setIf(config_, "scale", scale_);
    yml::setIf(config_, "bias", bias_);
    yml::setIf(config_, "Kp", kp_);
    yml::setIf(config_, "Kd", kd_);
  }
}

PositionActuator::PositionActuator(const NeuroPolicySpec &policy_spec, const std::string &name)
    : Actuator(policy_spec, name) {
  target_joint_pos_.setZero(policy_spec.dof);
  last_target_joint_pos_id_ = registerProvision("last_target_joint_pos", target_joint_pos_.size());
}

bool PositionActuator::reset() {
  is_first_update_ = true;
  return true;
}

bool PositionActuator::update(const LowState &low_state, ControlRequests &requests, FieldMap &context) {
  if (is_first_update_) {
    // Initialize last target position to current position
    for (Eigen::Index i{}; i < target_joint_pos_.size(); ++i) {
      target_joint_pos_[i] = low_state.motor_state[i].q;
    }
    is_first_update_ = false;
  }
  context[last_target_joint_pos_id_] = target_joint_pos_;
  return true;
}

void PositionActuator::setLowCmd(LowCmd &cmd, cArrXf action) {
  target_joint_pos_ = scale_.cwiseProduct(action) + bias_;
  for (Eigen::Index i{}; i < target_joint_pos_.size(); ++i) {
    cmd[i].q   = target_joint_pos_[i];
    cmd[i].dq  = 0.0F;
    cmd[i].tor = 0.0F;
    cmd[i].Kp  = kp_[i];
    cmd[i].Kd  = kd_[i];
  }
}

VelocityActuator::VelocityActuator(const NeuroPolicySpec &policy_spec, const std::string &name)
    : Actuator(policy_spec, name) {
  target_joint_vel_         = bias_;
  last_target_joint_vel_id_ = registerProvision("last_target_joint_vel", target_joint_vel_.size());
}

bool VelocityActuator::reset() {
  target_joint_vel_ = bias_;
  return true;
}

bool VelocityActuator::update(const LowState &low_state, ControlRequests &requests, FieldMap &context) {
  context[last_target_joint_vel_id_] = target_joint_vel_;
  return true;
}

void VelocityActuator::setLowCmd(LowCmd &cmd, cArrXf action) {
  target_joint_vel_ = scale_.cwiseProduct(action) + bias_;
  for (Eigen::Index i{}; i < target_joint_vel_.size(); ++i) {
    cmd[i].q   = 0.0F;
    cmd[i].dq  = target_joint_vel_[i];
    cmd[i].tor = 0.0F;
    cmd[i].Kp  = kp_[i];
    cmd[i].Kd  = kd_[i];
  }
}

TorqueActuator::TorqueActuator(const NeuroPolicySpec &policy_spec, const std::string &name)
    : Actuator(policy_spec, name) {
  target_joint_tor_         = bias_;
  last_target_joint_tor_id_ = registerProvision("last_target_joint_tor", target_joint_tor_.size());
}

bool TorqueActuator::reset() {
  target_joint_tor_ = bias_;
  return true;
}

bool TorqueActuator::update(const LowState &low_state, ControlRequests &requests, FieldMap &context) {
  context[last_target_joint_tor_id_] = target_joint_tor_;
  return true;
}

void TorqueActuator::setLowCmd(LowCmd &cmd, cArrXf action) {
  target_joint_tor_ = scale_.cwiseProduct(action) + bias_;
  for (Eigen::Index i{}; i < target_joint_tor_.size(); ++i) {
    cmd[i].q   = 0.0F;
    cmd[i].dq  = 0.0F;
    cmd[i].tor = target_joint_tor_[i];
    cmd[i].Kp  = kp_[i];
    cmd[i].Kd  = kd_[i];
  }
}

// clang-format off
const std::map<std::string, HybridActuator::Mode> HybridActuator::kModeMap = {
    {"position", HybridActuator::Mode::kPosition},
    {"velocity", HybridActuator::Mode::kVelocity},
    {"torque",   HybridActuator::Mode::kTorque},
};
// clang-format on

HybridActuator::HybridActuator(const NeuroPolicySpec &policy_spec, const std::string &name)
    : Actuator(policy_spec, name) {
  if (yml::hasValue(config_, "parameters")) {
    yml::Node parameters = config_["parameters"];
    for (const auto &node : config_["parameters"]) {
      modes_.push_back(lookupMap(yml::readAs<std::string>(node, "mode"), kModeMap));
    }
  } else {
    yml::assertSequenceOfSize(config_, "mode", policy_spec.dof);
    for (const auto &node : config_["mode"]) {
      modes_.push_back(lookupMap(yml::readAs<std::string>(node), kModeMap));
    }
  }

  joint_command_         = bias_;
  last_joint_command_id_ = registerProvision("last_joint_command", joint_command_.size());
}

bool HybridActuator::reset() {
  joint_command_ = bias_;
  return true;
}

bool HybridActuator::update(const LowState &low_state, ControlRequests &requests, FieldMap &context) {
  context[last_joint_command_id_] = joint_command_;
  return true;
}

void HybridActuator::setLowCmd(LowCmd &cmd, cArrXf action) {
  joint_command_ = scale_.cwiseProduct(action) + bias_;
  for (Eigen::Index i{}; i < joint_command_.size(); ++i) {
    cmd[i].q   = 0.0F;
    cmd[i].dq  = 0.0F;
    cmd[i].tor = 0.0F;
    switch (modes_[i]) {
      case Mode::kPosition:
        cmd[i].q = joint_command_[i];
        break;
      case Mode::kVelocity:
        cmd[i].dq = joint_command_[i];
        break;
      case Mode::kTorque:
        cmd[i].tor = joint_command_[i];
        break;
      default:
        STEPIT_UNREACHABLE();
        break;
    }
    cmd[i].Kp = kp_[i];
    cmd[i].Kd = kd_[i];
  }
}

STEPIT_REGISTER_ACTUATOR(position, kDefPriority, Actuator::make<PositionActuator>);
STEPIT_REGISTER_ACTUATOR(velocity, kDefPriority, Actuator::make<VelocityActuator>);
STEPIT_REGISTER_ACTUATOR(torque, kDefPriority, Actuator::make<TorqueActuator>);
STEPIT_REGISTER_ACTUATOR(hybrid, kDefPriority, Actuator::make<HybridActuator>);
}  // namespace neuro_policy
}  // namespace stepit

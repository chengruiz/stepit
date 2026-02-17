#include <stepit/robot.h>

namespace stepit {
RobotSpec::RobotSpec(const YAML::Node &config) {
  yml::setTo(config, "name", robot_name);
  yml::setTo(config, "joint_names", joint_names);
  yml::setTo(config, "foot_names", foot_names);
  dof      = joint_names.size();
  num_legs = foot_names.size();
  yml::setTo(config, "comm_freq", comm_freq);

  kp.resize(dof);
  kd.resize(dof);
  stuck_threshold.resize(dof, deg2rad(20.0F));
  standing_cfg.resize(dof);
  lying_cfg.resize(dof);

  yml::setTo(config, "kp", kp);
  yml::setTo(config, "kd", kd);
  yml::setIf(config, "stuck_threshold", stuck_threshold);

  if (auto node = config["safety"]) {
    yml::setIf(node, "enabled", safety.enabled);
    yml::setIf(node, "roll", safety.roll);
    yml::setIf(node, "pitch", safety.pitch);
  }

  yml::setIf(config, "resetting_time", resetting_time);
  yml::setIf(config, "standing_up_time", standing_up_time);
  yml::setIf(config, "lying_down_time", lying_down_time);
  yml::setIf(config, "returning_to_standing_time", returning_to_standing_time);

  yml::setTo(config, "standing_cfg", standing_cfg);
  yml::setTo(config, "lying_cfg", lying_cfg);
  yml::setTo(config, "auto_damped_mode", auto_damped_mode);
  yml::setIf(config, "kd_damped_mode", kd_damped_mode);
}

template <typename T>
void reorderInplace(std::vector<T> &values, const std::vector<std::size_t> &order, const char *name) {
  if (order.empty()) return;
  STEPIT_ASSERT_EQ(values.size(), order.size(), "{} size mismatch.", name);
  std::vector<T> reordered(values.size());
  for (std::size_t i{}; i < order.size(); ++i) {
    reordered[i] = std::move(values[order[i]]);
  }
  values = std::move(reordered);
}

RobotApiReorderingWrapper::RobotApiReorderingWrapper(const std::string &exposed_name, const std::string &wrapped_name,
                                                     std::vector<std::size_t> joint_order,
                                                     std::vector<std::size_t> foot_order,
                                                     std::vector<bool> joint_reversed)
    : wrapped_(RobotApi::make(wrapped_name)),
      joint_order_(std::move(joint_order)),
      foot_order_(std::move(foot_order)),
      joint_reversed_(std::move(joint_reversed)) {
  spec_ = wrapped_->getSpec();
  validateOrder(joint_order_, getDoF(), "joint_order");
  if (not foot_order_.empty()) validateOrder(foot_order_, getNumLegs(), "foot_order");
  STEPIT_ASSERT(joint_reversed_.empty() or joint_reversed_.size() == getDoF(),
                "joint_reversed size must be either 0 or match DoF");

  spec_.robot_name = exposed_name;
  reorderInplace(spec_.joint_names, joint_order_, "joint_names");
  reorderInplace(spec_.foot_names, foot_order_, "foot_names");
  reorderInplace(spec_.kp, joint_order_, "kp");
  reorderInplace(spec_.kd, joint_order_, "kd");
  reorderInplace(spec_.stuck_threshold, joint_order_, "stuck_threshold");
  reorderInplace(spec_.standing_cfg, joint_order_, "standing_cfg");
  reorderInplace(spec_.lying_cfg, joint_order_, "lying_cfg");
  for (std::size_t i{}; i < getDoF(); ++i) {
    if (not joint_reversed_.empty() and joint_reversed_[i]) {
      spec_.standing_cfg[i] *= -1;
      spec_.lying_cfg[i] *= -1;
    }
  }
}

void RobotApiReorderingWrapper::setSend(const LowCmd &cmd_msg) {
  LowCmd reordered(cmd_msg);
  for (std::size_t i{}; i < getDoF(); ++i) {
    reordered[joint_order_[i]] = cmd_msg[i];
    if (not joint_reversed_.empty() and joint_reversed_[i]) {
      reordered[joint_order_[i]].q *= -1;
      reordered[joint_order_[i]].dq *= -1;
      reordered[joint_order_[i]].tor *= -1;
    }
  }
  wrapped_->setSend(reordered);
}

void RobotApiReorderingWrapper::getRecv(LowState &state_msg) {
  wrapped_->getRecv(state_msg);
  reorderInplace(state_msg.motor_state, joint_order_, "motor_state");
  reorderInplace(state_msg.foot_force, foot_order_, "foot_force");
  for (std::size_t i{}; i < getDoF(); ++i) {
    if (not joint_reversed_.empty() and joint_reversed_[i]) {
      state_msg.motor_state[i].q *= -1;
      state_msg.motor_state[i].dq *= -1;
      state_msg.motor_state[i].tor *= -1;
    }
  }
}

void RobotApiReorderingWrapper::validateOrder(const std::vector<std::size_t> &order, std::size_t expected_size,
                                              const char *name) {
  STEPIT_ASSERT_EQ(order.size(), expected_size, "{} size mismatch.", name);

  std::vector<bool> seen(expected_size, false);
  for (std::size_t index : order) {
    STEPIT_ASSERT(index < expected_size, "{} index out of range.", name);
    STEPIT_ASSERT(not seen[index], "{} contains duplicate indices.", name);
    seen[index] = true;
  }
}
}  // namespace stepit

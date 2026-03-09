#include <stepit/robot.h>

namespace stepit {
RobotSpec::RobotSpec(const yml::Node &config) {
  config["name"].to(robot_name);
  config["joint_names"].to(joint_names);
  config["foot_names"].to(foot_names);
  dof      = joint_names.size();
  num_legs = foot_names.size();
  config["comm_freq"].to(comm_freq);

  kp.resize(dof);
  kd.resize(dof);
  stuck_threshold.resize(dof, deg2rad(20.0F));
  standing_cfg.resize(dof);
  lying_cfg.resize(dof);

  config[config.getDefinedKey({"stiffness", "kp", "Kp"})].to(kp);
  config[config.getDefinedKey({"damping", "kd", "Kd"})].to(kd);
  config["stuck_threshold"].to(stuck_threshold, true);

  const auto safety_node = config["safety"];
  if (safety_node.hasValue()) {
    safety_node["enabled"].to(safety.enabled, true);
    safety_node["roll"].to(safety.roll, true);
    safety_node["pitch"].to(safety.pitch, true);
  }

  config["resetting_time"].to(resetting_time, true);
  config["standing_up_time"].to(standing_up_time, true);
  config["lying_down_time"].to(lying_down_time, true);
  config["returning_to_standing_time"].to(returning_to_standing_time, true);

  config["standing_cfg"].to(standing_cfg);
  config["lying_cfg"].to(lying_cfg);
  config["auto_damped_mode"].to(auto_damped_mode);
  config["kd_damped_mode"].to(kd_damped_mode, true);
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

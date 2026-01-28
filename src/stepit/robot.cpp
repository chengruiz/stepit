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
}  // namespace stepit

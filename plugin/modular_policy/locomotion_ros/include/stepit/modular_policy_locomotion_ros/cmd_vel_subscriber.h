#ifndef STEPIT_MODULAR_POLICY_LOCOMOTION_ROS_CMD_VEL_SUBSCRIBER_H_
#define STEPIT_MODULAR_POLICY_LOCOMOTION_ROS_CMD_VEL_SUBSCRIBER_H_

#include <geometry_msgs/Twist.h>
#include <topic_tools/shape_shifter.h>

#include <stepit/modular_policy/locomotion/cmd_vel_source.h>
#include <stepit/ros/node_handle.h>

namespace stepit {
namespace modular_policy {
class CmdVelSubscriber : public CmdVelSource {
 public:
  CmdVelSubscriber(const ModularPolicySpec &policy_spec, const ModuleSpec &module_spec);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;
  void exit() override;

 private:
  void callback(const ros::MessageEvent<const topic_tools::ShapeShifter> &event);
  void handleControlRequest(ControlRequest request) override;

  std::mutex mutex_;
  ros::Subscriber cmd_vel_sub_;
  float timeout_threshold_{0.1F};
  bool default_subscriber_enabled_{false};
  publisher::StatusRegistration::Ptr subscribing_status_;

  std::atomic<bool> subscriber_enabled_{false};
  ros::Time cmd_vel_stamp_;
  geometry_msgs::Twist cmd_vel_msg_;
};
}  // namespace modular_policy
}  // namespace stepit

#endif  // STEPIT_MODULAR_POLICY_LOCOMOTION_ROS_CMD_VEL_SUBSCRIBER_H_

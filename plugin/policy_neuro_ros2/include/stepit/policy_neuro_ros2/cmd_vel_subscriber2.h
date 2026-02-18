#ifndef STEPIT_NEURO_POLICY_ROS2_CMD_VEL_SUBSCRIBER2_H_
#define STEPIT_NEURO_POLICY_ROS2_CMD_VEL_SUBSCRIBER2_H_

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <stepit/policy_neuro/cmd_vel_source.h>

namespace stepit::neuro_policy {
class CmdVelSubscriber2 : public CmdVelSource {
 public:
  CmdVelSubscriber2(const PolicySpec &policy_spec, const std::string &home_dir);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;
  void exit() override;

 private:
  void handleControlRequest(ControlRequest request) override;
  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void twistStampedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  rclcpp::SubscriptionBase::SharedPtr cmd_vel_sub_{nullptr};
  float timeout_threshold_{0.1F};
  bool default_subscriber_enabled_{false};
  publisher::StatusRegistration::Ptr subscribing_status_;

  std::atomic<bool> subscriber_enabled_{false};
  rclcpp::Time cmd_vel_stamp_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Twist cmd_vel_msg_;
};
}  // namespace stepit::neuro_policy

#endif  // STEPIT_NEURO_POLICY_ROS2_CMD_VEL_SUBSCRIBER2_H_

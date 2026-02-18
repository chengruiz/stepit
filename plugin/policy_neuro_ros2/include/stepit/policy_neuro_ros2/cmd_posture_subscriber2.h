#ifndef STEPIT_NEURO_POLICY_ROS2_CMD_POSTURE_SUBSCRIBER2_H_
#define STEPIT_NEURO_POLICY_ROS2_CMD_POSTURE_SUBSCRIBER2_H_

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <stepit/policy_neuro/cmd_posture_source.h>

namespace stepit::neuro_policy {
class CmdRollSubscriber2 : public CmdRollSource {
 public:
  CmdRollSubscriber2(const PolicySpec &policy_spec, const std::string &home_dir);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;
  void exit() override;

 private:
  void handleControlRequest(ControlRequest request) override;
  void float32Callback(const std_msgs::msg::Float32::SharedPtr msg);
  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void twistStampedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  std::mutex mutex_;
  rclcpp::SubscriptionBase::SharedPtr cmd_roll_sub_{nullptr};
  float timeout_threshold_{0.1F};
  bool default_subscriber_enabled_{false};
  publisher::StatusRegistration::Ptr subscribing_status_;

  std::atomic<bool> subscriber_enabled_{false};
  rclcpp::Time cmd_roll_stamp_{0, 0, RCL_ROS_TIME};
  float cmd_roll_msg_{};
};

class CmdPitchSubscriber2 : public CmdPitchSource {
 public:
  CmdPitchSubscriber2(const PolicySpec &policy_spec, const std::string &home_dir);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;
  void exit() override;

 private:
  void handleControlRequest(ControlRequest request) override;
  void float32Callback(const std_msgs::msg::Float32::SharedPtr msg);
  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void twistStampedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  std::mutex mutex_;
  rclcpp::SubscriptionBase::SharedPtr cmd_pitch_sub_{nullptr};
  float timeout_threshold_{0.1F};
  bool default_subscriber_enabled_{false};
  publisher::StatusRegistration::Ptr subscribing_status_;

  std::atomic<bool> subscriber_enabled_{false};
  rclcpp::Time cmd_pitch_stamp_{0, 0, RCL_ROS_TIME};
  float cmd_pitch_msg_;
};

class CmdHeightSubscriber2 : public CmdHeightSource {
 public:
  CmdHeightSubscriber2(const PolicySpec &policy_spec, const std::string &home_dir);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;
  void exit() override;

 private:
  void handleControlRequest(ControlRequest request) override;
  void float32Callback(const std_msgs::msg::Float32::SharedPtr msg);
  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void twistStampedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  std::mutex mutex_;
  rclcpp::SubscriptionBase::SharedPtr cmd_height_sub_{nullptr};
  float timeout_threshold_{0.1F};
  bool default_subscriber_enabled_{false};
  publisher::StatusRegistration::Ptr subscribing_status_;

  std::atomic<bool> subscriber_enabled_{false};
  rclcpp::Time cmd_height_stamp_{0, 0, RCL_ROS_TIME};
  float cmd_height_msg_;
};
}  // namespace stepit::neuro_policy

#endif  // STEPIT_NEURO_POLICY_ROS2_CMD_POSTURE_SUBSCRIBER2_H_

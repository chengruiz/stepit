#ifndef STEPIT_NEURO_POLICY_ROS_CMD_POSTURE_SUBSCRIBER_H_
#define STEPIT_NEURO_POLICY_ROS_CMD_POSTURE_SUBSCRIBER_H_

#include <topic_tools/shape_shifter.h>

#include <stepit/policy_neuro/cmd_posture_source.h>
#include <stepit/ros/node_handle.h>

namespace stepit {
namespace neuro_policy {
class CmdRollSubscriber : public CmdRollSource {
 public:
  CmdRollSubscriber(const NeuroPolicySpec &policy_spec, const std::string &name);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;
  void exit() override;

 private:
  void callback(const ros::MessageEvent<const topic_tools::ShapeShifter> &event);
  void handleControlRequest(ControlRequest request) override;

  std::mutex mutex_;
  ros::Subscriber cmd_roll_sub_;
  float timeout_threshold_{0.1F};
  bool default_subscriber_enabled_{false};
  publisher::StatusRegistration::Ptr subscribing_status_;

  std::atomic<bool> subscriber_enabled_{false};
  ros::Time cmd_roll_stamp_;
  float cmd_roll_msg_{};
};

class CmdPitchSubscriber : public CmdPitchSource {
 public:
  CmdPitchSubscriber(const NeuroPolicySpec &policy_spec, const std::string &name);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;
  void exit() override;

 private:
  void callback(const ros::MessageEvent<const topic_tools::ShapeShifter> &event);
  void handleControlRequest(ControlRequest request) override;

  std::mutex mutex_;
  ros::Subscriber cmd_pitch_sub_;
  float timeout_threshold_{0.1F};
  bool default_subscriber_enabled_{false};
  publisher::StatusRegistration::Ptr subscribing_status_;

  std::atomic<bool> subscriber_enabled_{false};
  ros::Time cmd_pitch_stamp_;
  float cmd_pitch_msg_;
};

class CmdHeightSubscriber : public CmdHeightSource {
 public:
  CmdHeightSubscriber(const NeuroPolicySpec &policy_spec, const std::string &name);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;
  void exit() override;

 private:
  void callback(const ros::MessageEvent<const topic_tools::ShapeShifter> &event);
  void handleControlRequest(ControlRequest request) override;

  std::mutex mutex_;
  ros::Subscriber cmd_height_sub_;
  float timeout_threshold_{0.1F};
  bool default_subscriber_enabled_{false};
  publisher::StatusRegistration::Ptr subscribing_status_;

  std::atomic<bool> subscriber_enabled_{false};
  ros::Time cmd_height_stamp_;
  float cmd_height_msg_;
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_ROS_CMD_POSTURE_SUBSCRIBER_H_

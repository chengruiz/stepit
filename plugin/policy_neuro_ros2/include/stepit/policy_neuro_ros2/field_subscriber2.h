#ifndef STEPIT_NEURO_POLICY_ROS2_FIELD_SUBSCRIBER2_H_
#define STEPIT_NEURO_POLICY_ROS2_FIELD_SUBSCRIBER2_H_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <stepit/policy.h>
#include <stepit/policy_neuro/field.h>

namespace stepit::neuro_policy {
class FieldSubscriber2 : public Module {
 public:
  FieldSubscriber2(const PolicySpec &policy_spec, const std::string &home_dir);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &result) override;

 private:
  void callback(std::size_t index, const std_msgs::msg::Float32MultiArray::SharedPtr msg);

  struct FieldData {
    FieldId id{};
    std::string name;
    std::string topic;
    std::size_t size;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber;
    float timeout_threshold{};

    bool received{false};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    VecXf data;
  };

  YAML::Node config_;
  std::vector<FieldData> fields_;
  std::mutex mutex_;
};
}  // namespace stepit::neuro_policy

#endif  // STEPIT_NEURO_POLICY_ROS2_FIELD_SUBSCRIBER2_H_

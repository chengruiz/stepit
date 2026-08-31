#ifndef STEPIT_MODULAR_POLICY_ROS2_FIELD_SUBSCRIBER2_H_
#define STEPIT_MODULAR_POLICY_ROS2_FIELD_SUBSCRIBER2_H_

#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <stepit/modular_policy/module.h>

namespace stepit::modular_policy {
class FieldSubscriber2 : public Module {
 public:
  FieldSubscriber2(const ModularPolicySpec &policy_spec, const ModuleSpec &module_spec);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;

 private:
  template <typename Scalar, typename Sequence>
  void storeField(std::size_t index, const Sequence &source);
  void float32Callback(std::size_t index, const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void int32Callback(std::size_t index, const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  void int64Callback(std::size_t index, const std_msgs::msg::Int64MultiArray::SharedPtr msg);
  void boolCallback(std::size_t index, const std_msgs::msg::UInt8MultiArray::SharedPtr msg);

  struct FieldData {
    FieldId id{};
    std::string name;
    std::string topic;
    DataType dtype{DataType::kUndefined};
    FieldSize size{};
    float timeout_threshold{};

    bool received{false};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    rclcpp::SubscriptionBase::SharedPtr subscriber;
  };

  // Keep subscription handles last so they are destroyed before callback state.
  std::mutex mutex_;
  FieldMap field_values_;
  std::vector<FieldData> fields_;
};
}  // namespace stepit::modular_policy

#endif  // STEPIT_MODULAR_POLICY_ROS2_FIELD_SUBSCRIBER2_H_

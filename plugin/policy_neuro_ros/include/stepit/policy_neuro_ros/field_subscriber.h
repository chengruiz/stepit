#ifndef STEPIT_NEURO_POLICY_ROS_FIELD_SUBSCRIBER_H_
#define STEPIT_NEURO_POLICY_ROS_FIELD_SUBSCRIBER_H_

#include <mutex>
#include <vector>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>

#include <stepit/policy_neuro/module.h>
#include <stepit/ros/node_handle.h>

namespace stepit {
namespace neuro_policy {
class FieldSubscriber : public Module {
 public:
  FieldSubscriber(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;

 private:
  template <typename Scalar, typename Sequence>
  void storeField(std::size_t index, const Sequence &source);
  void float32Callback(std::size_t index, const std_msgs::Float32MultiArray::ConstPtr &msg);
  void int32Callback(std::size_t index, const std_msgs::Int32MultiArray::ConstPtr &msg);
  void int64Callback(std::size_t index, const std_msgs::Int64MultiArray::ConstPtr &msg);
  void boolCallback(std::size_t index, const std_msgs::UInt8MultiArray::ConstPtr &msg);

  struct FieldData {
    FieldId id{};
    std::string name;
    std::string topic;
    DataType dtype{DataType::kUndefined};
    FieldSize size{};
    float timeout_threshold{};

    bool received{false};
    ros::Time stamp;
    ros::Subscriber subscriber;
  };

  // Keep subscription handles last so they are destroyed before callback state.
  std::mutex mutex_;
  FieldMap field_values_;
  std::vector<FieldData> fields_;
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_ROS_FIELD_SUBSCRIBER_H_

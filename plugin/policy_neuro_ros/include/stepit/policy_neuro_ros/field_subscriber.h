#ifndef STEPIT_NEURO_POLICY_ROS_FIELD_SUBSCRIBER_H_
#define STEPIT_NEURO_POLICY_ROS_FIELD_SUBSCRIBER_H_

#include <std_msgs/Float32MultiArray.h>

#include <stepit/policy.h>
#include <stepit/policy_neuro/field.h>
#include <stepit/ros/node_handle.h>

namespace stepit {
namespace neuro_policy {
class FieldSubscriber : public Module {
 public:
  FieldSubscriber(const PolicySpec &policy_spec, const std::string &home_dir);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &result) override;

 private:
  void callback(std::size_t index, const std_msgs::Float32MultiArray::ConstPtr &msg);

  struct FieldData {
    FieldId id{};
    std::string name;
    std::string topic;
    std::size_t size;
    ros::Subscriber subscriber;
    float timeout_threshold{};

    bool received{false};
    ros::Time stamp;
    VecXf data;
  };

  YAML::Node config_;
  std::vector<FieldData> fields_;
  std::mutex mutex_;
};
}  // namespace neuro_policy
}  // namespace stepit

#endif  // STEPIT_NEURO_POLICY_ROS_FIELD_SUBSCRIBER_H_

#ifndef STEPIT_ROS2_PUBLISHER_H_
#define STEPIT_ROS2_PUBLISHER_H_

#include <map>
#include <string>
#include <variant>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <stepit/publisher.h>

namespace stepit {
class Ros2Publisher : public Publisher {
 public:
  Ros2Publisher();
  void publishStatus() override;
  void publishLowLevel(const RobotSpec &spec, const LowState &state, const LowCmd &cmd) override;
  void publishArray(const std::string &name, const void *data, DataType dtype, std::size_t size) override;

 private:
  class ArrayPublisher {
   public:
    ArrayPublisher(std::string name, DataType dtype);
    void publish(const void *data, DataType dtype, std::size_t size);

   private:
    template <typename Message, typename Scalar>
    class TypedPublisher {
     public:
      TypedPublisher(const rclcpp::Node::SharedPtr &node, const std::string &name);
      void publish(const void *data, std::size_t size);

     private:
      typename rclcpp::Publisher<Message>::SharedPtr publisher_;
    };

    using Float32Publisher = TypedPublisher<std_msgs::msg::Float32MultiArray, float>;
    using Int32Publisher   = TypedPublisher<std_msgs::msg::Int32MultiArray, std::int32_t>;
    using Int64Publisher   = TypedPublisher<std_msgs::msg::Int64MultiArray, std::int64_t>;
    using BoolPublisher    = TypedPublisher<std_msgs::msg::UInt8MultiArray, std::uint8_t>;
    using PublisherVariant = std::variant<Float32Publisher, Int32Publisher, Int64Publisher, BoolPublisher>;

    static PublisherVariant makePublisher(const rclcpp::Node::SharedPtr &node, const std::string &name, DataType dtype);

    std::string name_;
    DataType dtype_;
    PublisherVariant publisher_;
  };

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  std::map<std::string, ArrayPublisher> array_publishers_;

  diagnostic_msgs::msg::DiagnosticStatus status_msg_;
  sensor_msgs::msg::JointState joint_msg_;
  sensor_msgs::msg::Imu imu_msg_;
  geometry_msgs::msg::Twist twist_msg_;
};
}  // namespace stepit

#endif  // STEPIT_ROS2_PUBLISHER_H_

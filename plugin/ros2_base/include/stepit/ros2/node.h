#ifndef STEPIT_ROS2_NODE_H_
#define STEPIT_ROS2_NODE_H_

#include <rclcpp/rclcpp.hpp>

#include <stepit/utils.h>

namespace stepit {
rclcpp::Node::SharedPtr &getNode();
rclcpp::QoS &getDefaultQoS();

std::string getTopicType(const std::string &topic_name, const std::string &default_type = "");

rclcpp::QoS parseQoS(const yml::Node &node);

struct TopicInfo {
  std::string name;
  std::string type;
  rclcpp::QoS qos{1};
};

TopicInfo parseTopicInfo(const yml::Node &node, const std::string &default_name = "",
                         const std::string &default_type = "");

inline double getElapsedTime(const rclcpp::Time &start_time) { return (getNode()->now() - start_time).seconds(); }
}  // namespace stepit

#endif  // STEPIT_ROS2_NODE_H_

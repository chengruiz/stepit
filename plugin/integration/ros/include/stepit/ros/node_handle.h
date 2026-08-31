#ifndef STEPIT_ROS_NODE_HANDLE_H_
#define STEPIT_ROS_NODE_HANDLE_H_

#include <ros/ros.h>

#include <stepit/utils.h>

namespace stepit {
ros::NodeHandle &getNodeHandle();

ros::TransportHints parseTransportHints(const yml::Node &node);

template <class M, class T>
ros::Subscriber makeSubscriber(const yml::Node &cfg, void (T::*fp)(M), T *obj, const std::string &default_topic = "") {
  if (default_topic.empty()) cfg["topic"].assertHasValue();
  std::string topic    = cfg["topic"].as<std::string>(default_topic);
  uint32_t queue_size  = cfg["queue_size"].as<uint32_t>(1UL);
  auto transport_hints = parseTransportHints(cfg["transport_hints"]);
  return getNodeHandle().subscribe(topic, queue_size, fp, obj, transport_hints);
}

template <class T>
ros::Publisher makePublisher(const yml::Node &cfg, const std::string &default_topic = "") {
  if (default_topic.empty()) cfg["topic"].assertHasValue();
  std::string topic   = cfg["topic"].as<std::string>(default_topic);
  uint32_t queue_size = cfg["queue_size"].as<uint32_t>(1UL);
  bool latch          = cfg["latch"].as<bool>(false);
  return getNodeHandle().advertise<T>(topic, queue_size, latch);
}

inline double getElapsedTime(const ros::Time &start_time) { return (ros::Time::now() - start_time).toSec(); }
}  // namespace stepit

#endif  // STEPIT_ROS_NODE_HANDLE_H_

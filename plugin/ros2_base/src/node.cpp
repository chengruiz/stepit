#include <stepit/ros2/node.h>

namespace stepit {
std::shared_ptr<rclcpp::Node> g_node;

const std::map<std::string, rclcpp::ReliabilityPolicy> kReliabilityPolicyMap = {
    {"reliable", rclcpp::ReliabilityPolicy::Reliable},
    {"best_effort", rclcpp::ReliabilityPolicy::BestEffort},
    {"system_default", rclcpp::ReliabilityPolicy::SystemDefault},
};

const std::map<std::string, rclcpp::DurabilityPolicy> kDurabilityPolicyMap = {
    {"volatile", rclcpp::DurabilityPolicy::Volatile},
    {"transient_local", rclcpp::DurabilityPolicy::TransientLocal},
    {"system_default", rclcpp::DurabilityPolicy::SystemDefault},
};

const std::map<std::string, rclcpp::HistoryPolicy> kHistoryPolicyMap = {
    {"keep_last", rclcpp::HistoryPolicy::KeepLast},
    {"keep_all", rclcpp::HistoryPolicy::KeepAll},
    {"system_default", rclcpp::HistoryPolicy::SystemDefault},
};

rclcpp::Node::SharedPtr &getNode() { return g_node; }

rclcpp::QoS &getDefaultQoS() {
  static rclcpp::QoS default_qos{rclcpp::SensorDataQoS()};
  static bool initialized = false;
  if (not initialized) {
    std::string reliability, durability, history;
    if (getenv("STEPIT_ROS2_QOS_RELIABILITY", reliability)) {
      toLowercaseInplace(reliability);
      auto reliability_policy = kReliabilityPolicyMap.find(reliability);
      STEPIT_ASSERT(reliability_policy != kReliabilityPolicyMap.end(), "Unknown QoS reliability '{}'.", reliability);
      default_qos.reliability(reliability_policy->second);
    }

    if (getenv("STEPIT_ROS2_QOS_DURABILITY", durability)) {
      toLowercaseInplace(durability);
      auto durability_policy = kDurabilityPolicyMap.find(durability);
      STEPIT_ASSERT(durability_policy != kDurabilityPolicyMap.end(), "Unknown QoS durability '{}'.", durability);
      default_qos.durability(durability_policy->second);
    }

    if (getenv("STEPIT_ROS2_QOS_HISTORY", history)) {
      char *end{nullptr};
      long value = std::strtol(history.c_str(), &end, 10);
      if (*end == '\0') {  // integer
        default_qos.keep_last(static_cast<std::size_t>(value));
      } else {
        toLowercaseInplace(history);
        auto history_policy = kHistoryPolicyMap.find(history);
        STEPIT_ASSERT(history_policy != kHistoryPolicyMap.end(), "Unknown QoS history '{}'.", history);
        default_qos.history(history_policy->second);
      }
    }
    initialized = true;
  }

  return default_qos;
}

std::string getTopicType(const std::string &topic_name, const std::string &default_type) {
  auto topic_info = getNode()->get_topic_names_and_types();
  auto it         = topic_info.find(topic_name);
  if (it == topic_info.end()) return default_type;
  const auto &topic_types = it->second;
  if (topic_types.empty()) return default_type;
  return topic_types[0];
}

rclcpp::QoS parseQoS(const yml::Node &node) {
  rclcpp::QoS value{getDefaultQoS()};
  if (not node.hasValue()) return value;

  const auto reliability_node = node["reliability"];
  if (reliability_node.hasValue()) {
    auto reliability = reliability_node.as<std::string>();
    toLowercaseInplace(reliability);
    auto reliability_policy = kReliabilityPolicyMap.find(reliability);
    STEPIT_ASSERT(reliability_policy != kReliabilityPolicyMap.end(), "Unknown QoS reliability '{}'.", reliability);
    value.reliability(reliability_policy->second);
  }

  const auto durability_node = node["durability"];
  if (durability_node.hasValue()) {
    auto durability = durability_node.as<std::string>();
    toLowercaseInplace(durability);
    auto durability_policy = kDurabilityPolicyMap.find(durability);
    STEPIT_ASSERT(durability_policy != kDurabilityPolicyMap.end(), "Unknown QoS durability '{}'.", durability);
    value.durability(durability_policy->second);
  }

  const auto history_node = node["history"];
  if (history_node.hasValue()) {
    if (history_node.isType<std::size_t>()) {
      value.keep_last(history_node.as<std::size_t>());
    } else {
      auto history = history_node.as<std::string>();
      toLowercaseInplace(history);
      auto history_policy = kHistoryPolicyMap.find(history);
      STEPIT_ASSERT(history_policy != kHistoryPolicyMap.end(), "Unknown QoS history '{}'.", history);
      value.history(history_policy->second);
    }
  }
  return value;
}

TopicInfo parseTopicInfo(const yml::Node &node, const std::string &default_name, const std::string &default_type) {
  TopicInfo info{};
  if (default_name.empty()) {
    node["topic"].to(info.name);
  } else {
    info.name = node["topic"].as<std::string>(default_name);
  }
  info.type = getTopicType(info.name, node["topic_type"].as<std::string>(default_type));
  info.qos  = parseQoS(node["qos"]);
  return info;
}
}  // namespace stepit

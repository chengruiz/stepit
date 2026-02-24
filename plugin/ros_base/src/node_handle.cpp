#include <stepit/ros/node_handle.h>

namespace stepit {
ros::NodeHandle &getNodeHandle() {
  static ros::NodeHandle nh;
  return nh;
}

ros::TransportHints parseTransportHints(const yml::Node &node) {
  ros::TransportHints value;
  if (not node) return value.tcpNoDelay();
  auto type = yml::readIf<std::string>(node, "type", "tcpnodelay");
  toLowercaseInplace(type);
  if (type == "reliable") {
    value.reliable();
  } else if (type == "unreliable") {
    value.unreliable();
  } else if (type == "tcp") {
    value.tcp();
  } else if (type == "udp") {
    value.udp();
  } else if (type == "tcpnodelay") {
    value.tcpNoDelay();
  } else {
    STEPIT_THROW("Unknown transport hint type '{}'.", type);
  }
  if (node["max_datagram_size"]) {
    auto size = yml::readAs<int>(node, "max_datagram_size");
    value.maxDatagramSize(size);
  }
  return value;
}
}  // namespace stepit

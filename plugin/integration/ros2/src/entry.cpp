#include <stepit/utils.h>
#include <stepit/ros2/node.h>

extern "C" {
int stepit_plugin_init(int &argc, char **argv) {
  if (not rclcpp::ok()) {
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::SigTerm);
  }
  std::string node_name{"stepit_ros2"};
  stepit::getenv("STEPIT_ROS2_NODE_NAME", node_name);
  stepit::getNode() = rclcpp::Node::make_shared(node_name);
  return 0;
}

void stepit_plugin_cleanup() {
  stepit::getNode().reset();
  rclcpp::shutdown();
}
}

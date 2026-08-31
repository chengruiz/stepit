#include <stepit/utils.h>
#include <stepit/ros/node_handle.h>

extern "C" {
int stepit_plugin_init(int &argc, char **argv) {
  std::string node_name{"stepit_ros"};
  stepit::getenv("STEPIT_ROS_NODE_NAME", node_name);
  ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
  stepit::getNodeHandle();
  return 0;
}
}

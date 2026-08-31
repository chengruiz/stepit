#include <stepit/ros/control.h>
#include <stepit/ros/node_handle.h>

namespace stepit {
RosMsgControl::RosMsgControl() {
  sub_ = getNodeHandle().subscribe<std_msgs::String>("control", 1, &RosMsgControl::msgCallback, this);
}

void RosMsgControl::msgCallback(const std_msgs::String::ConstPtr &msg) {
  received_ = true;
  put(msg->data);
}

RosSrvControl::RosSrvControl() {
  srv_ = getNodeHandle().advertiseService("control", &RosSrvControl::srvCallback, this);
}

bool RosSrvControl::srvCallback(stepit_ros_msgs::Control::Request &req, stepit_ros_msgs::Control::Response &res) {
  received_     = true;
  auto response = put(req.request).get();
  res.status    = response.status;
  res.message   = response.message;
  return true;
}

STEPIT_REGISTER_CTRLINPUT(ros_msg, kDefPriority, ControlInput::make<RosMsgControl>);
STEPIT_REGISTER_CTRLINPUT(ros_srv, kDefPriority, ControlInput::make<RosSrvControl>);
}  // namespace stepit

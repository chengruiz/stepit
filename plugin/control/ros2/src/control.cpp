#include <stepit/ros2/control.h>
#include <stepit/ros2/node.h>

namespace stepit {
Ros2MsgControl::Ros2MsgControl() {
  sub_ = getNode()->create_subscription<std_msgs::msg::String>(
      "control", getDefaultQoS(), std::bind(&Ros2MsgControl::msgCallback, this, std::placeholders::_1));
}

void Ros2MsgControl::msgCallback(const std_msgs::msg::String::SharedPtr msg) {
  received_ = true;
  put(msg->data);
}

Ros2SrvControl::Ros2SrvControl() {
  srv_ = getNode()->create_service<ControlSrv>(
      "control", std::bind(&Ros2SrvControl::srvCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void Ros2SrvControl::srvCallback(const ControlSrv::Request::SharedPtr req, ControlSrv::Response::SharedPtr res) {
  received_     = true;
  auto response = put(req->request).get();
  res->status   = response.status;
  res->message  = response.message;
}

STEPIT_REGISTER_CTRLINPUT(ros2_msg, kDefPriority, ControlInput::make<Ros2MsgControl>);
STEPIT_REGISTER_CTRLINPUT(ros2_srv, kDefPriority, ControlInput::make<Ros2SrvControl>);
}  // namespace stepit

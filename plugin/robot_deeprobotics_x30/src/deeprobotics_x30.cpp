#include <stepit/logging.h>
#include <stepit/robot/deeprobotics_x30.h>

namespace stepit {
DeepRoboticsX30Api::DeepRoboticsX30Api() : RobotApi(kRobotName), state_msg_(&low_state_sub_.getRecvState()) {
  std::string host_ip{"192.168.1.120"};
  getenv("STEPIT_HOST_IP", host_ip);
  low_cmd_pub_ = std::make_unique<x30::SendToRobot>(host_ip);
}

void DeepRoboticsX30Api::getControl(bool enable) {
  if (enable) {
    low_state_sub_.startWork();
    low_cmd_pub_->robot_state_init();
    low_cmd_pub_->control_get(ABLE);
  } else {
    low_cmd_pub_->control_get(UNABLE);
  }
}

void DeepRoboticsX30Api::setSend(const LowCmd &cmd_msg) {
  for (std::size_t i{}; i < getDoF(); ++i) {
    cmd_msg_.joint_cmd[i].pos = cmd_msg[i].q;
    cmd_msg_.joint_cmd[i].vel = cmd_msg[i].dq;
    cmd_msg_.joint_cmd[i].tor = cmd_msg[i].tor;
    cmd_msg_.joint_cmd[i].kp  = cmd_msg[i].Kp;
    cmd_msg_.joint_cmd[i].kd  = cmd_msg[i].Kd;
  }
  low_cmd_pub_->set_send(cmd_msg_);
}

void copyImuData(const x30::ImuDataSDK &src, LowState::IMU &dst) {
  auto quaternion      = Quatf::fromEulerAngles(dst.rpy);
  dst.quaternion[0]    = quaternion.w();
  dst.quaternion[1]    = quaternion.x();
  dst.quaternion[2]    = quaternion.y();
  dst.quaternion[3]    = quaternion.z();
  dst.rpy[0]           = deg2rad(src.roll);
  dst.rpy[1]           = deg2rad(src.pitch);
  dst.rpy[2]           = deg2rad(src.yaw);
  dst.gyroscope[0]     = src.omega_x;
  dst.gyroscope[1]     = src.omega_y;
  dst.gyroscope[2]     = src.omega_z;
  dst.accelerometer[0] = src.acc_x;
  dst.accelerometer[1] = src.acc_y;
  dst.accelerometer[2] = src.acc_z;
}

void DeepRoboticsX30Api::getRecv(LowState &state_msg) {
  copyImuData(state_msg_->imu, state_msg.imu);
  for (std::size_t i{}; i < getDoF(); ++i) {
    state_msg.motor_state[i].q   = state_msg_->joint_data[i].pos;
    state_msg.motor_state[i].dq  = state_msg_->joint_data[i].vel;
    state_msg.motor_state[i].tor = state_msg_->joint_data[i].tor;
  }
  for (std::size_t i{}; i < getNumLegs(); ++i) {
    state_msg.foot_force[i] = cmVec3f(state_msg_->contact_force.data() + i * 3).norm();
  }
  state_msg.tick = state_msg_->tick;
}

constexpr std::array<std::size_t, 12> kJointOrderDeepRobotics2Unitree{3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};
constexpr std::array<std::size_t, 4> kFootOrderDeepRobotics2Unitree{1, 0, 3, 2};

STEPIT_REGISTER_ROBOTAPI(x30, kDefPriority, RobotApi::make<DeepRoboticsX30Api>);
STEPIT_REGISTER_ROBOTAPI(x30u, kDefPriority, []() {
  return std::make_unique<RobotApiReorderingWrapper>(  //
      "x30u",                                          // Exposed name
      "x30",                                           // Wrapped name
      array2vector(kJointOrderDeepRobotics2Unitree),   // Joint 3-5 <-> 0-2, 9-11 <-> 6-8
      array2vector(kFootOrderDeepRobotics2Unitree),    // Foot FR <-> FL, HR <-> HL
      std::vector<bool>(12, true)                      // All joints reversed
  );
});
}  // namespace stepit

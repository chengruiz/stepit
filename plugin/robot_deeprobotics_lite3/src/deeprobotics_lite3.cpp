#include <stepit/logging.h>
#include <stepit/robot/deeprobotics_lite3.h>

namespace stepit {
DeepRoboticsLite3Api::DeepRoboticsLite3Api() : RobotApi(kRobotName) {
  std::string host_ip{"192.168.1.120"};
  getenv("STEPIT_HOST_IP", host_ip);

  low_cmd_pub_   = std::make_unique<Sender>(host_ip, 43893);
  low_state_sub_ = std::make_unique<Receiver>();
  low_state_sub_->RegisterCallBack([this](uint32_t code) {
    if (code == 0x0906) ++tick_;
  });
  state_msg_ = &low_state_sub_->GetState();
}
DeepRoboticsLite3Api::~DeepRoboticsLite3Api() {}

void DeepRoboticsLite3Api::getControl(bool enable) {
  if (enable) {
    low_state_sub_->StartWork();
    low_cmd_pub_->RobotStateInit();
    low_cmd_pub_->ControlGet(SDK);
  } else {
    low_cmd_pub_->ControlGet(ROBOT);
  }
}

void DeepRoboticsLite3Api::setSend(const LowCmd &cmd_msg) {
  for (std::size_t i{}; i < getDoF(); ++i) {
    cmd_msg_.joint_cmd[i].position = cmd_msg[i].q;
    cmd_msg_.joint_cmd[i].velocity = cmd_msg[i].dq;
    cmd_msg_.joint_cmd[i].kp       = cmd_msg[i].Kp;
    cmd_msg_.joint_cmd[i].kd       = cmd_msg[i].Kd;
    cmd_msg_.joint_cmd[i].torque   = cmd_msg[i].tor;
  }
  low_cmd_pub_->SendCmd(cmd_msg_);
}

void DeepRoboticsLite3Api::getRecv(LowState &state_msg) {
  auto quaternion                = Quatf::fromEulerAngles(state_msg.imu.rpy);
  state_msg.imu.quaternion[0]    = quaternion.w();
  state_msg.imu.quaternion[1]    = quaternion.x();
  state_msg.imu.quaternion[2]    = quaternion.y();
  state_msg.imu.quaternion[3]    = quaternion.z();
  state_msg.imu.rpy[0]           = deg2rad(state_msg_->imu.angle_roll);
  state_msg.imu.rpy[1]           = deg2rad(state_msg_->imu.angle_pitch);
  state_msg.imu.rpy[2]           = deg2rad(state_msg_->imu.angle_yaw);
  state_msg.imu.gyroscope[0]     = state_msg_->imu.angular_velocity_roll;
  state_msg.imu.gyroscope[1]     = state_msg_->imu.angular_velocity_pitch;
  state_msg.imu.gyroscope[2]     = state_msg_->imu.angular_velocity_yaw;
  state_msg.imu.accelerometer[0] = state_msg_->imu.acc_x;
  state_msg.imu.accelerometer[1] = state_msg_->imu.acc_y;
  state_msg.imu.accelerometer[2] = state_msg_->imu.acc_z;

  for (std::size_t i{}; i < getDoF(); ++i) {
    state_msg.motor_state[i].q   = state_msg_->joint_data.joint_data[i].position;
    state_msg.motor_state[i].dq  = state_msg_->joint_data.joint_data[i].velocity;
    state_msg.motor_state[i].tor = state_msg_->joint_data.joint_data[i].torque;
  }
  for (std::size_t i{}; i < getNumLegs(); ++i) {
    state_msg.foot_force[i] = static_cast<float>(cmVec3d(state_msg_->contact_force.leg_force + i * 3).norm());
  }
  state_msg.tick = tick_;
}

STEPIT_REGISTER_ROBOTAPI(lite3, kDefPriority, RobotApi::make<DeepRoboticsLite3Api>);
}  // namespace stepit

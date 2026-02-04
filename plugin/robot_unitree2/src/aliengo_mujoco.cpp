#include <stepit/robot/unitree2/aliengo_mujoco.h>

namespace stepit {
AliengoMujocoApi::AliengoMujocoApi() : RobotApi(kRobotName), low_state_(getDoF(), getNumLegs()) {
  low_cmd_.head()[0]    = 0xFE;
  low_cmd_.head()[1]    = 0xEF;
  low_cmd_.level_flag() = 0xFF;
  low_cmd_.gpio()       = 0;
  for (auto &motor_cmd : low_cmd_.motor_cmd()) {
    motor_cmd.mode() = kGo2MotorServoMode;
    motor_cmd.q()    = kPosStop;
    motor_cmd.dq()   = kVelStop;
    motor_cmd.kp()   = 0;
    motor_cmd.kd()   = 0;
    motor_cmd.tau()  = 0;
  }
}

void AliengoMujocoApi::getControl(bool enable) {
  if (enable) {
    Unitree2ServiceClient::initialize();
    Unitree2ServiceClient::deactivate();

    low_cmd_pub_   = std::make_shared<u2_sdk::ChannelPublisher<u2_msg::LowCmd_>>("rt/lowcmd");
    low_state_sub_ = std::make_shared<u2_sdk::ChannelSubscriber<u2_msg::LowState_>>("rt/lowstate");
    low_cmd_pub_->InitChannel();
    low_state_sub_->InitChannel([this](const void *msg) { callback(static_cast<const u2_msg::LowState_ *>(msg)); }, 1);
  }
}

void AliengoMujocoApi::setSend(LowCmd &cmd_msg) {
  for (std::size_t i{}; i < getDoF(); ++i) {
    low_cmd_.motor_cmd()[i].q()   = cmd_msg[i].q;
    low_cmd_.motor_cmd()[i].dq()  = cmd_msg[i].dq;
    low_cmd_.motor_cmd()[i].kp()  = cmd_msg[i].Kp;
    low_cmd_.motor_cmd()[i].kd()  = cmd_msg[i].Kd;
    low_cmd_.motor_cmd()[i].tau() = cmd_msg[i].tor;
  }
}

void AliengoMujocoApi::getRecv(LowState &state_msg) {
  std::lock_guard<std::mutex> _(mutex_);
  state_msg = low_state_;
}

void AliengoMujocoApi::send() {
  low_cmd_.crc() = crc32core(reinterpret_cast<uint32_t *>(&low_cmd_), (sizeof(u2_msg::LowCmd_) >> 2) - 1);
  low_cmd_pub_->Write(low_cmd_);
}

void AliengoMujocoApi::callback(const u2_msg::LowState_ *msg) {
  std::lock_guard<std::mutex> _(mutex_);
  low_state_.imu.rpy           = msg->imu_state().rpy();
  low_state_.imu.quaternion    = msg->imu_state().quaternion();
  low_state_.imu.accelerometer = msg->imu_state().accelerometer();
  low_state_.imu.gyroscope     = msg->imu_state().gyroscope();

  for (std::size_t i{}; i < getDoF(); ++i) {
    low_state_.motor_state[i].q   = msg->motor_state()[i].q();
    low_state_.motor_state[i].dq  = msg->motor_state()[i].dq();
    low_state_.motor_state[i].tor = msg->motor_state()[i].tau_est();
  }
  for (std::size_t i{}; i < getNumLegs(); ++i) {
    low_state_.foot_force[i] = msg->foot_force()[i];
  }
  low_state_.tick = msg->tick();
}

STEPIT_REGISTER_ROBOTAPI(aliengo_mujoco, kDefPriority, RobotApi::make<AliengoMujocoApi>);
}  // namespace stepit

#include <stepit/robot/unitree2/g1.h>

namespace stepit {
G1DoF15Api::G1DoF15Api() : RobotApi(kRobotName), low_state_(getDoF(), getNumLegs()) {
  low_cmd_.mode_pr(0);  // {0:PR, 1:AB}
  low_cmd_.mode_machine(5);

  for (auto &motor_cmd : low_cmd_.motor_cmd()) {
    motor_cmd.mode() = 0x01;
    motor_cmd.q()    = kPosStop;
    motor_cmd.dq()   = kVelStop;
    motor_cmd.tau()  = 0;
    motor_cmd.kp()   = 0;
    motor_cmd.kd()   = 0;
  }
  default_arm_pos_.resize(kArmDoF);
  default_arm_pos_ << 0, 0.62, 0, 1.04, 0, 0, 0, 0, -0.62, 0, 1.04, 0, 0, 0;
}

void G1DoF15Api::getControl(bool enable) {
  if (enable) {
    Unitree2ServiceClient::initialize();
    low_cmd_pub_   = std::make_shared<u2_sdk::ChannelPublisher<hg_msg::LowCmd_>>("rt/lowcmd");
    low_state_sub_ = std::make_shared<u2_sdk::ChannelSubscriber<hg_msg::LowState_>>("rt/lowstate");
    low_cmd_pub_->InitChannel();
    low_state_sub_->InitChannel([this](const void *msg) { callback(static_cast<const hg_msg::LowState_ *>(msg)); }, 1);
  }
}

void G1DoF15Api::setSend(const LowCmd &cmd_msg) {
  setArmCommands();

  for (int i{}; i < kLegDoF; ++i) {
    low_cmd_.motor_cmd()[i].mode() = 1;
    low_cmd_.motor_cmd()[i].q()    = cmd_msg[i].q;
    low_cmd_.motor_cmd()[i].dq()   = cmd_msg[i].dq;
    low_cmd_.motor_cmd()[i].kp()   = cmd_msg[i].Kp;
    low_cmd_.motor_cmd()[i].kd()   = cmd_msg[i].Kd;
    low_cmd_.motor_cmd()[i].tau()  = cmd_msg[i].tor;
  }
  for (int i{}; i < kArmDoF; i++) {
    low_cmd_.motor_cmd()[i + kLegDoF].mode() = 1;
    low_cmd_.motor_cmd()[i + kLegDoF].q()    = arm_cmd_[i].q;
    low_cmd_.motor_cmd()[i + kLegDoF].dq()   = arm_cmd_[i].dq;
    low_cmd_.motor_cmd()[i + kLegDoF].kp()   = arm_cmd_[i].Kp;
    low_cmd_.motor_cmd()[i + kLegDoF].kd()   = arm_cmd_[i].Kd;
    low_cmd_.motor_cmd()[i + kLegDoF].tau()  = arm_cmd_[i].tor;
  }
}

void G1DoF15Api::getRecv(LowState &state_msg) {
  std::lock_guard<std::mutex> _(mutex_);
  state_msg = low_state_;
}

void G1DoF15Api::send() { 
  fillLowCmdCrc(low_cmd_);
  low_cmd_pub_->Write(low_cmd_);
}

void G1DoF15Api::callback(const hg_msg::LowState_ *msg) {
  std::lock_guard<std::mutex> _(mutex_);
  low_state_.imu.rpy           = msg->imu_state().rpy();
  low_state_.imu.quaternion    = msg->imu_state().quaternion();
  low_state_.imu.accelerometer = msg->imu_state().accelerometer();
  low_state_.imu.gyroscope     = msg->imu_state().gyroscope();

  for (int i{}; i < kLegDoF; ++i) {
    low_state_.motor_state[i].q   = msg->motor_state()[i].q();
    low_state_.motor_state[i].dq  = msg->motor_state()[i].dq();
    low_state_.motor_state[i].tor = msg->motor_state()[i].tau_est();
  }
  for (int i{}; i < kArmDoF; ++i) {
    arm_state_[i].q   = msg->motor_state()[i + kLegDoF].q();
    arm_state_[i].dq  = msg->motor_state()[i + kLegDoF].dq();
    arm_state_[i].tor = msg->motor_state()[i + kLegDoF].tau_est();
  }
  low_state_.tick = msg->tick();
}

void G1DoF15Api::setArmCommands() {
  for (int i{}; i < kArmDoF; ++i) {
    float curr_q    = arm_state_[i].q;
    arm_cmd_[i].q   = curr_q + clamp(default_arm_pos_[i] - curr_q, -0.05F, 0.05F);
    arm_cmd_[i].dq  = 0.0F;
    arm_cmd_[i].Kp  = 60.0F;
    arm_cmd_[i].Kd  = 1.0F;
    arm_cmd_[i].tor = 0.0F;
  }
}

G1DoF23Api::G1DoF23Api() : RobotApi(kRobotName), low_state_(getDoF(), getNumLegs()) {
  low_cmd_.mode_pr(0);  // {0:PR, 1:AB}
  low_cmd_.mode_machine(5);

  for (auto &motor_cmd : low_cmd_.motor_cmd()) {
    motor_cmd.mode() = 0x01;
    motor_cmd.q()    = kPosStop;
    motor_cmd.dq()   = kVelStop;
    motor_cmd.tau()  = 0;
    motor_cmd.kp()   = 0;
    motor_cmd.kd()   = 0;
  }
  des_arm_pos_.setZero(6);
}

void G1DoF23Api::getControl(bool enable) {
  if (enable) {
    Unitree2ServiceClient::initialize();
    low_cmd_pub_   = std::make_shared<u2_sdk::ChannelPublisher<hg_msg::LowCmd_>>("rt/lowcmd");
    low_state_sub_ = std::make_shared<u2_sdk::ChannelSubscriber<hg_msg::LowState_>>("rt/lowstate");
    low_cmd_pub_->InitChannel();
    low_state_sub_->InitChannel([this](const void *msg) { callback(static_cast<const hg_msg::LowState_ *>(msg)); }, 1);
  }
}

void G1DoF23Api::setSend(const LowCmd &cmd_msg) {
  ArrXf target_arm_pos{6};
  for (int i{}; i < 6; ++i) {
    target_arm_pos[i] = curr_arm_pos_[i] + clamp(des_arm_pos_[i] - curr_arm_pos_[i], -0.05F, 0.05F);
  }

  for (int i{}; i < 19; ++i) {
    low_cmd_.motor_cmd()[i].mode() = 1;
    low_cmd_.motor_cmd()[i].q()    = cmd_msg[i].q;
    low_cmd_.motor_cmd()[i].dq()   = cmd_msg[i].dq;
    low_cmd_.motor_cmd()[i].kp()   = cmd_msg[i].Kp;
    low_cmd_.motor_cmd()[i].kd()   = cmd_msg[i].Kd;
    low_cmd_.motor_cmd()[i].tau()  = cmd_msg[i].tor;
  }
  for (int i{19}; i < 22; i++) {
    low_cmd_.motor_cmd()[i].mode() = 1;
    low_cmd_.motor_cmd()[i].q()    = target_arm_pos[i - 19];
    low_cmd_.motor_cmd()[i].dq()   = 0;
    low_cmd_.motor_cmd()[i].kp()   = 60;
    low_cmd_.motor_cmd()[i].kd()   = 1.;
    low_cmd_.motor_cmd()[i].tau()  = 0;
  }
  for (int i{22}; i < 26; i++) {
    low_cmd_.motor_cmd()[i].mode() = 1;
    low_cmd_.motor_cmd()[i].q()    = cmd_msg[i - 3].q;
    low_cmd_.motor_cmd()[i].dq()   = cmd_msg[i - 3].dq;
    low_cmd_.motor_cmd()[i].kp()   = cmd_msg[i - 3].Kp;
    low_cmd_.motor_cmd()[i].kd()   = cmd_msg[i - 3].Kd;
    low_cmd_.motor_cmd()[i].tau()  = cmd_msg[i - 3].tor;
  }
  for (int i{26}; i < 29; i++) {
    low_cmd_.motor_cmd()[i].mode() = 1;
    low_cmd_.motor_cmd()[i].q()    = target_arm_pos[i - 23];
    low_cmd_.motor_cmd()[i].dq()   = 0;
    low_cmd_.motor_cmd()[i].kp()   = 60;
    low_cmd_.motor_cmd()[i].kd()   = 1.;
    low_cmd_.motor_cmd()[i].tau()  = 0;
  }
}

void G1DoF23Api::getRecv(LowState &state_msg) {
  std::lock_guard<std::mutex> _(mutex_);
  state_msg = low_state_;
}

void G1DoF23Api::send() { 
  fillLowCmdCrc(low_cmd_);
  low_cmd_pub_->Write(low_cmd_); 
}

void G1DoF23Api::callback(const hg_msg::LowState_ *msg) {
  std::lock_guard<std::mutex> _(mutex_);
  low_state_.imu.rpy           = msg->imu_state().rpy();
  low_state_.imu.quaternion    = msg->imu_state().quaternion();
  low_state_.imu.accelerometer = msg->imu_state().accelerometer();
  low_state_.imu.gyroscope     = msg->imu_state().gyroscope();

  for (int i{}; i < 19; ++i) {
    low_state_.motor_state[i].q   = msg->motor_state()[i].q();
    low_state_.motor_state[i].dq  = msg->motor_state()[i].dq();
    low_state_.motor_state[i].tor = msg->motor_state()[i].tau_est();
  }
  for (int i{22}; i < 26; ++i) {
    low_state_.motor_state[i - 3].q   = msg->motor_state()[i].q();
    low_state_.motor_state[i - 3].dq  = msg->motor_state()[i].dq();
    low_state_.motor_state[i - 3].tor = msg->motor_state()[i].tau_est();
  }
  low_state_.tick = msg->tick();

  curr_arm_pos_[0] = msg->motor_state()[19].q();
  curr_arm_pos_[1] = msg->motor_state()[20].q();
  curr_arm_pos_[2] = msg->motor_state()[21].q();
  curr_arm_pos_[3] = msg->motor_state()[26].q();
  curr_arm_pos_[4] = msg->motor_state()[27].q();
  curr_arm_pos_[5] = msg->motor_state()[28].q();
}

G1DoF29Api::G1DoF29Api() : RobotApi(kRobotName), low_state_(getDoF(), getNumLegs()) {
  low_cmd_.mode_pr(0);  // {0:PR, 1:AB}
  low_cmd_.mode_machine(5);

  for (auto &motor_cmd : low_cmd_.motor_cmd()) {
    motor_cmd.mode() = 0x01;
    motor_cmd.q()    = kPosStop;
    motor_cmd.dq()   = kVelStop;
    motor_cmd.tau()  = 0;
    motor_cmd.kp()   = 0;
    motor_cmd.kd()   = 0;
  }
}

void G1DoF29Api::getControl(bool enable) {
  if (enable) {
    Unitree2ServiceClient::initialize();
    low_cmd_pub_   = std::make_shared<u2_sdk::ChannelPublisher<hg_msg::LowCmd_>>("rt/lowcmd");
    low_state_sub_ = std::make_shared<u2_sdk::ChannelSubscriber<hg_msg::LowState_>>("rt/lowstate");
    low_cmd_pub_->InitChannel();
    low_state_sub_->InitChannel([this](const void *msg) { callback(static_cast<const hg_msg::LowState_ *>(msg)); }, 1);
  }
}

void G1DoF29Api::setSend(const LowCmd &cmd_msg) {
  for (int i{}; i < getDoF(); ++i) {
    low_cmd_.motor_cmd()[i].mode() = 1;
    low_cmd_.motor_cmd()[i].q()    = cmd_msg[i].q;
    low_cmd_.motor_cmd()[i].dq()   = cmd_msg[i].dq;
    low_cmd_.motor_cmd()[i].kp()   = cmd_msg[i].Kp;
    low_cmd_.motor_cmd()[i].kd()   = cmd_msg[i].Kd;
    low_cmd_.motor_cmd()[i].tau()  = cmd_msg[i].tor;
  }
}

void G1DoF29Api::getRecv(LowState &state_msg) {
  std::lock_guard<std::mutex> _(mutex_);
  state_msg = low_state_;
}

void G1DoF29Api::send() { 
  fillLowCmdCrc(low_cmd_);
  low_cmd_pub_->Write(low_cmd_);
}

void G1DoF29Api::callback(const hg_msg::LowState_ *msg) {
  std::lock_guard<std::mutex> _(mutex_);
  low_state_.imu.rpy           = msg->imu_state().rpy();
  low_state_.imu.quaternion    = msg->imu_state().quaternion();
  low_state_.imu.accelerometer = msg->imu_state().accelerometer();
  low_state_.imu.gyroscope     = msg->imu_state().gyroscope();

  for (int i{}; i < getDoF(); ++i) {
    low_state_.motor_state[i].q   = msg->motor_state()[i].q();
    low_state_.motor_state[i].dq  = msg->motor_state()[i].dq();
    low_state_.motor_state[i].tor = msg->motor_state()[i].tau_est();
  }
  low_state_.tick = msg->tick();
}

STEPIT_REGISTER_ROBOTAPI(g1_15dof, kDefPriority, RobotApi::make<G1DoF15Api>);
STEPIT_REGISTER_ROBOTAPI(g1_23dof, kDefPriority, RobotApi::make<G1DoF23Api>);
STEPIT_REGISTER_ROBOTAPI(g1_29dof, kDefPriority, RobotApi::make<G1DoF29Api>);
}  // namespace stepit

#include <stepit/robot/unitree2/a2.h>

namespace stepit {
A2Api::A2Api() : RobotApi(kRobotName), low_state_(getDoF(), getNumLegs()) {
  STEPIT_ASSERT_EQ(getDoF(), 12, "A2 robot config must describe 12 joints.");

  low_cmd_.mode_pr(0);

  for (auto &motor_cmd : low_cmd_.motor_cmd()) {
    motor_cmd.mode() = 0;
    motor_cmd.q()    = kPosStop;
    motor_cmd.dq()   = kVelStop;
    motor_cmd.tau()  = 0;
    motor_cmd.kp()   = 0;
    motor_cmd.kd()   = 0;
  }

  for (std::size_t i{}; i < getDoF(); ++i) {
    low_cmd_.motor_cmd()[i].mode() = 1;
  }
}

void A2Api::getControl(bool enable) {
  if (not enable) return;

  Unitree2ServiceSwitcher::serviceSwitch("ai_sport", false);

  low_cmd_pub_   = std::make_shared<u2_sdk::ChannelPublisher<hg_msg::LowCmd_>>("rt/lowcmd");
  low_state_sub_ = std::make_shared<u2_sdk::ChannelSubscriber<hg_msg::LowState_>>("rt/lowstate");
  low_cmd_pub_->InitChannel();
  low_state_sub_
      ->InitChannel([this](const void *msg) { lowStateCallback(static_cast<const hg_msg::LowState_ *>(msg)); }, 1);
}

void A2Api::setSend(const LowCmd &cmd_msg) {
  low_cmd_.mode_machine(mode_machine_.load(std::memory_order_relaxed));
  for (std::size_t i{}; i < getDoF(); ++i) {
    low_cmd_.motor_cmd()[i].mode() = 1;
    low_cmd_.motor_cmd()[i].q()    = cmd_msg[i].q;
    low_cmd_.motor_cmd()[i].dq()   = cmd_msg[i].dq;
    low_cmd_.motor_cmd()[i].kp()   = cmd_msg[i].Kp;
    low_cmd_.motor_cmd()[i].kd()   = cmd_msg[i].Kd;
    low_cmd_.motor_cmd()[i].tau()  = cmd_msg[i].tor;
  }
}

void A2Api::getRecv(LowState &state_msg) {
  std::lock_guard<std::mutex> _(mutex_);
  state_msg = low_state_;
}

void A2Api::send() {
  fillLowCmdCrc(low_cmd_);
  low_cmd_pub_->Write(low_cmd_);
}

void A2Api::lowStateCallback(const hg_msg::LowState_ *msg) {
  std::lock_guard<std::mutex> _(mutex_);
  mode_machine_.store(msg->mode_machine(), std::memory_order_relaxed);

  low_state_.imu.rpy           = msg->imu_state().rpy();
  low_state_.imu.quaternion    = msg->imu_state().quaternion();
  low_state_.imu.accelerometer = msg->imu_state().accelerometer();
  low_state_.imu.gyroscope     = msg->imu_state().gyroscope();

  for (std::size_t i{}; i < getDoF(); ++i) {
    low_state_.motor_state[i].q   = msg->motor_state()[i].q();
    low_state_.motor_state[i].dq  = msg->motor_state()[i].dq();
    low_state_.motor_state[i].tor = msg->motor_state()[i].tau_est();
  }
  low_state_.tick = msg->tick();
}

STEPIT_REGISTER_ROBOTAPI(a2, kDefPriority, RobotApi::make<A2Api>);
}  // namespace stepit

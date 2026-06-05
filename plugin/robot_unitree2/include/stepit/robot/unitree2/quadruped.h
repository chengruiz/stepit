#ifndef STEPIT_ROBOT_UNITREE2_QUADRUPED_H_
#define STEPIT_ROBOT_UNITREE2_QUADRUPED_H_

#include <memory>
#include <mutex>

#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <stepit/robot.h>
#include <stepit/robot/unitree2/common.h>

namespace u2_msg = unitree_go::msg::dds_;

namespace stepit {
template <typename Spec>
class UnitreeQuadrupedApi : public RobotApi {
 public:
  UnitreeQuadrupedApi() : RobotApi(Spec::robotName()), low_state_(getDoF(), getNumLegs()) {
    low_cmd_.head()[0]    = 0xFE;
    low_cmd_.head()[1]    = 0xEF;
    low_cmd_.level_flag() = 0xFF;
    low_cmd_.gpio()       = 0;
    for (auto &motor_cmd : low_cmd_.motor_cmd()) {
      motor_cmd.mode() = Spec::kMotorServoMode;
      motor_cmd.q()    = kPosStop;
      motor_cmd.dq()   = kVelStop;
      motor_cmd.kp()   = 0;
      motor_cmd.kd()   = 0;
      motor_cmd.tau()  = 0;
    }
  }

  void getControl(bool enable) override {
    if (not enable) return;

    Unitree2Dds::initialize();
    Unitree2MotionSwitcher::deactivate();

    low_cmd_pub_   = std::make_shared<u2_sdk::ChannelPublisher<u2_msg::LowCmd_>>("rt/lowcmd");
    low_state_sub_ = std::make_shared<u2_sdk::ChannelSubscriber<u2_msg::LowState_>>("rt/lowstate");
    low_cmd_pub_->InitChannel();
    low_state_sub_->InitChannel([this](const void *msg) { callback(static_cast<const u2_msg::LowState_ *>(msg)); }, 1);
  }

  void setSend(const LowCmd &cmd_msg) override {
    for (std::size_t i{}; i < getDoF(); ++i) {
      low_cmd_.motor_cmd()[i].q()   = cmd_msg[i].q;
      low_cmd_.motor_cmd()[i].dq()  = cmd_msg[i].dq;
      low_cmd_.motor_cmd()[i].kp()  = cmd_msg[i].Kp;
      low_cmd_.motor_cmd()[i].kd()  = cmd_msg[i].Kd;
      low_cmd_.motor_cmd()[i].tau() = cmd_msg[i].tor;
    }
  }

  void getRecv(LowState &state_msg) override {
    std::lock_guard<std::mutex> _(mutex_);
    state_msg = low_state_;
  }

  void send() override {
    fillLowCmdCrc(low_cmd_);
    low_cmd_pub_->Write(low_cmd_);
  }

  void recv() override {}

 private:
  void callback(const u2_msg::LowState_ *msg) {
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

  u2_sdk::ChannelPublisherPtr<u2_msg::LowCmd_> low_cmd_pub_;
  u2_sdk::ChannelSubscriberPtr<u2_msg::LowState_> low_state_sub_;
  u2_msg::LowCmd_ low_cmd_{};
  LowState low_state_;
  std::mutex mutex_;
};

struct Go2Spec {
  static const char *robotName() { return "go2"; }
  static constexpr uint8_t kMotorServoMode = kGo2MotorServoMode;
};

struct Go2WSpec {
  static const char *robotName() { return "go2w"; }
  static constexpr uint8_t kMotorServoMode = kGo2MotorServoMode;
};

struct B2Spec {
  static const char *robotName() { return "b2"; }
  static constexpr uint8_t kMotorServoMode = kB2MotorServoMode;
};

struct AliengoSimSpec {
  static const char *robotName() { return "aliengo_sim"; }
  static constexpr uint8_t kMotorServoMode = kGo2MotorServoMode;
};

using Go2Api = UnitreeQuadrupedApi<Go2Spec>;
using Go2WApi = UnitreeQuadrupedApi<Go2WSpec>;
using B2Api = UnitreeQuadrupedApi<B2Spec>;
using AliengoSimApi = UnitreeQuadrupedApi<AliengoSimSpec>;
}  // namespace stepit

#endif  // STEPIT_ROBOT_UNITREE2_QUADRUPED_H_

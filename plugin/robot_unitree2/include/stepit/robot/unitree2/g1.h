#ifndef STEPIT_ROBOT_UNITREE2_G1_H_
#define STEPIT_ROBOT_UNITREE2_G1_H_

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <stepit/robot.h>
#include <stepit/utils.h>
#include <stepit/robot/unitree2/common.h>

namespace hg_msg = unitree_hg::msg::dds_;

namespace stepit {
class G1DoF15Api : public RobotApi {
 public:
  G1DoF15Api();
  void getControl(bool enable) override;
  void setSend(const LowCmd &cmd_msg) override;
  void getRecv(LowState &state_msg) override;
  void send() override;
  void recv() override {}

  static constexpr std::size_t kLegDoF    = 15;
  static constexpr std::size_t kArmDoF    = 14;
  static constexpr const char *kRobotName = "g1_15dof";

 private:
  void callback(const hg_msg::LowState_ *msg);

 protected:
  virtual void setArmCommands();

  u2_sdk::ChannelPublisherPtr<hg_msg::LowCmd_> low_cmd_pub_;
  u2_sdk::ChannelSubscriberPtr<hg_msg::LowState_> low_state_sub_;
  hg_msg::LowCmd_ low_cmd_{};
  LowState low_state_;
  std::mutex mutex_;

  ArrXf default_arm_pos_;
  std::array<MotorState, kArmDoF> arm_state_{};
  LowCmd arm_cmd_{kArmDoF};
};

class G1DoF23Api final : public RobotApi {
 public:
  G1DoF23Api();
  void getControl(bool enable) override;
  void setSend(const LowCmd &cmd_msg) override;
  void getRecv(LowState &state_msg) override;
  void send() override;
  void recv() override {}

  static constexpr const char *kRobotName = "g1_23dof";

 private:
  void callback(const hg_msg::LowState_ *msg);

  u2_sdk::ChannelPublisherPtr<hg_msg::LowCmd_> low_cmd_pub_;
  u2_sdk::ChannelSubscriberPtr<hg_msg::LowState_> low_state_sub_;
  hg_msg::LowCmd_ low_cmd_{};
  LowState low_state_;
  std::mutex mutex_;

  ArrXf curr_arm_pos_{6};
  ArrXf des_arm_pos_{6};
};

class G1DoF29Api final : public RobotApi {
 public:
  G1DoF29Api();
  void getControl(bool enable) override;
  void setSend(const LowCmd &cmd_msg) override;
  void getRecv(LowState &state_msg) override;
  void send() override;
  void recv() override {}

  static constexpr const char *kRobotName = "g1_29dof";

 private:
  void callback(const hg_msg::LowState_ *msg);

  u2_sdk::ChannelPublisherPtr<hg_msg::LowCmd_> low_cmd_pub_;
  u2_sdk::ChannelSubscriberPtr<hg_msg::LowState_> low_state_sub_;
  hg_msg::LowCmd_ low_cmd_{};
  LowState low_state_;
  std::mutex mutex_;
};
}  // namespace stepit

#endif  // STEPIT_ROBOT_UNITREE2_G1_H_

#ifndef STEPIT_ROBOT_UNITREE2_ALIENGO_MUJOCO_H_
#define STEPIT_ROBOT_UNITREE2_ALIENGO_MUJOCO_H_

#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <mutex>

#include <stepit/robot.h>
#include <stepit/robot/unitree2/common.h>

namespace u2_msg = unitree_go::msg::dds_;

namespace stepit {
class AliengoMujocoApi final : public RobotApi {
 public:
  AliengoMujocoApi();
  void getControl(bool enable) override;
  void setSend(LowCmd &cmd_msg) override;
  void getRecv(LowState &state_msg) override;
  void send() override;
  void recv() override {}

  static constexpr const char *kRobotName = "aliengo_mujoco";

 private:
  void callback(const u2_msg::LowState_ *msg);

  u2_sdk::ChannelPublisherPtr<u2_msg::LowCmd_> low_cmd_pub_;
  u2_sdk::ChannelSubscriberPtr<u2_msg::LowState_> low_state_sub_;
  u2_msg::LowCmd_ low_cmd_{};
  LowState low_state_;
  std::mutex mutex_;
};
}  // namespace stepit

#endif  // STEPIT_ROBOT_UNITREE2_ALIENGO_MUJOCO_H_

#ifndef STEPIT_ROBOT_UNITREE2_B2_H_
#define STEPIT_ROBOT_UNITREE2_B2_H_

#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <stepit/robot.h>
#include <stepit/utils.h>
#include <stepit/robot/unitree2/common.h>

namespace u2_msg = unitree_go::msg::dds_;

namespace stepit {
class B2Api final : public RobotApi {
 public:
  B2Api();
  void getControl(bool enable) override;
  void setSend(const LowCmd &cmd_msg) override;
  void getRecv(LowState &state_msg) override;
  void send() override;
  void recv() override {}

  static constexpr const char *kRobotName = "b2";

 private:
  void callback(const u2_msg::LowState_ *msg);
  std::size_t joint_vel_filter_window_{0};

  u2_sdk::ChannelPublisherPtr<u2_msg::LowCmd_> low_cmd_pub_;
  u2_sdk::ChannelSubscriberPtr<u2_msg::LowState_> low_state_sub_;
  u2_msg::LowCmd_ low_cmd_{};
  LowState low_state_;
  RingBuffer<LowState> low_state_buf_;
  std::mutex mutex_;
};
}  // namespace stepit

#endif  // STEPIT_ROBOT_UNITREE2_B2_H_

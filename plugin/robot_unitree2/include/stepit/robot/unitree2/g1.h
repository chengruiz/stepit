#ifndef STEPIT_ROBOT_UNITREE2_G1_H_
#define STEPIT_ROBOT_UNITREE2_G1_H_

#include <atomic>

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <stepit/robot.h>
#include <stepit/utils.h>
#include <stepit/robot/unitree2/common.h>

namespace hg_msg = unitree_hg::msg::dds_;

namespace stepit {
class G1Api final : public RobotApi {
 public:
  G1Api();
  void getControl(bool enable) override;
  void setSend(const LowCmd &cmd_msg) override;
  void getRecv(LowState &state_msg) override;
  void send() override;
  void recv() override {}

  static constexpr const char *kRobotName = "g1";

 private:
  void callback(const hg_msg::LowState_ *msg);

  u2_sdk::ChannelPublisherPtr<hg_msg::LowCmd_> low_cmd_pub_;
  u2_sdk::ChannelSubscriberPtr<hg_msg::LowState_> low_state_sub_;

  std::mutex mutex_;
  std::atomic<std::uint8_t> mode_machine_{0};
  hg_msg::LowCmd_ low_cmd_{};
  std::array<bool, 29> motor_enabled_{};
  LowState low_state_;
};
}  // namespace stepit

#endif  // STEPIT_ROBOT_UNITREE2_G1_H_

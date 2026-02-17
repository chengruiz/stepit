#ifndef STEPIT_ROBOT_DEEPROBOTICS_X30_H_
#define STEPIT_ROBOT_DEEPROBOTICS_X30_H_

#include <parse_cmd.h>
#include <send_to_robot.h>

#include <stepit/robot.h>

namespace stepit {
class DeepRoboticsX30Api final : public RobotApi {
 public:
  DeepRoboticsX30Api();
  void getControl(bool enable) override;
  void setSend(const LowCmd &cmd_msg) override;
  void getRecv(LowState &state_msg) override;
  void send() override {}
  void recv() override {}
  static constexpr const char *kRobotName = "x30";

 private:
  std::unique_ptr<x30::SendToRobot> low_cmd_pub_;
  x30::ParseCommand low_state_sub_;
  x30::RobotDataSDK *state_msg_{nullptr};
  x30::RobotCmdSDK cmd_msg_{};
};
}  // namespace stepit

#endif  // STEPIT_ROBOT_DEEPROBOTICS_X30_H_

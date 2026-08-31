#ifndef STEPIT_ROBOT_UNITREE_H_
#define STEPIT_ROBOT_UNITREE_H_

#include <unitree_legged_sdk/unitree_legged_sdk.h>

#include <stepit/robot.h>

namespace stepit {
namespace unitree = UNITREE_LEGGED_SDK;

class UnitreeApi final : public RobotApi {
 public:
  UnitreeApi();
  void getControl(bool enable) override;
  void setSend(const LowCmd &cmd_msg) override;
  void getRecv(LowState &state_msg) override;
  void send() override;
  void recv() override;

  static constexpr const char *getRobotName();

 private:
  unitree::UDP udp_;
  unitree::LowCmd low_cmd_{};
  unitree::LowState low_state_{};
};
}  // namespace stepit

#endif  // STEPIT_ROBOT_UNITREE_H_

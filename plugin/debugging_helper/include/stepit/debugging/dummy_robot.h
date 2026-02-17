#ifndef STEPIT_DEBUGGING_DUMMY_ROBOT_H_
#define STEPIT_DEBUGGING_DUMMY_ROBOT_H_

#include <stepit/robot.h>

namespace stepit {
class DummyRobotApi final : public RobotApi {
 public:
  DummyRobotApi();
  void getControl(bool enable) override {}
  void setSend(const LowCmd &cmd_msg) override;
  void getRecv(LowState &state_msg) override;
  void send() override {}
  void recv() override {}

  static constexpr const char *kRobotName = "dummy";

 private:
  LowState low_state_;
  TimePoint start_time_;
};
}  // namespace stepit

#endif  // STEPIT_DEBUGGING_DUMMY_ROBOT_H_

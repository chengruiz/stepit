#include <stepit/debugging/dummy_robot.h>

namespace stepit {
DummyRobotApi::DummyRobotApi()
    : RobotApi(kRobotName), low_state_(getDoF(), getNumLegs()), start_time_(SteadyClock::now()) {}

void DummyRobotApi::setSend(const LowCmd &cmd_msg) {
  for (std::size_t i{}; i < getDoF(); ++i) {
    low_state_.motor_state[i].q   = cmd_msg[i].q;
    low_state_.motor_state[i].dq  = cmd_msg[i].dq;
    low_state_.motor_state[i].tor = cmd_msg[i].tor;
  }
}

void DummyRobotApi::getRecv(LowState &state_msg) {
  low_state_.tick = static_cast<uint32_t>(getElapsedTime<MSec>(start_time_));
  state_msg       = low_state_;
}

STEPIT_REGISTER_ROBOTAPI(dummy, kMinPriority, RobotApi::make<DummyRobotApi>);
}  // namespace stepit

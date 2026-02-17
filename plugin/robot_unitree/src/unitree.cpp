#include <stepit/robot/unitree.h>

#define UNITREE_ALIENGO 0
#define UNITREE_GO1     1
#define UNITREE_B1      2

#ifndef STEPIT_UNITREE_ROBOT
#error "STEPIT_UNITREE_ROBOT not defined."
#endif  // STEPIT_UNITREE_ROBOT

namespace stepit {
UnitreeApi::UnitreeApi()
    : RobotApi(getRobotName()),
      udp_(unitree::LOWLEVEL
#if STEPIT_UNITREE_ROBOT != UNITREE_ALIENGO
           ,
           8090, "192.168.123.10", 8007
#endif
      ) {
  udp_.InitCmdData(low_cmd_);
  low_cmd_.levelFlag = unitree::LOWLEVEL;
  for (auto &motor_cmd : low_cmd_.motorCmd) {
    motor_cmd.mode = 0x0A;  // motor switch to servo (PMSM) mode
    motor_cmd.q    = unitree::PosStopF;
    motor_cmd.dq   = unitree::VelStopF;
    motor_cmd.Kp   = 0.;
    motor_cmd.Kd   = 0.;
    motor_cmd.tau  = 0.;
  }
}

void UnitreeApi::getControl(bool enable) {
  if (enable) {
    udp_.SetSend(low_cmd_);
    udp_.Send();
  }
}

void UnitreeApi::setSend(const LowCmd &cmd_msg) {
  for (std::size_t i{}; i < getDoF(); ++i) {
    low_cmd_.motorCmd[i].q   = cmd_msg[i].q;
    low_cmd_.motorCmd[i].dq  = cmd_msg[i].dq;
    low_cmd_.motorCmd[i].Kp  = cmd_msg[i].Kp;
    low_cmd_.motorCmd[i].Kd  = cmd_msg[i].Kd;
    low_cmd_.motorCmd[i].tau = cmd_msg[i].tor;
  }
  udp_.SetSend(low_cmd_);
}

void UnitreeApi::getRecv(LowState &state_msg) {
  udp_.GetRecv(low_state_);
  state_msg.tick              = low_state_.tick;
  state_msg.imu.quaternion    = low_state_.imu.quaternion;
  state_msg.imu.gyroscope     = low_state_.imu.gyroscope;
  state_msg.imu.accelerometer = low_state_.imu.accelerometer;
  state_msg.imu.rpy           = low_state_.imu.rpy;

  for (std::size_t i{}; i < getDoF(); ++i) {
    state_msg.motor_state[i].q   = low_state_.motorState[i].q;
    state_msg.motor_state[i].dq  = low_state_.motorState[i].dq;
    state_msg.motor_state[i].tor = low_state_.motorState[i].tauEst;
  }
  for (std::size_t i{}; i < getNumLegs(); ++i) {
    state_msg.foot_force[i] = low_state_.footForce[i];
  }
}

void UnitreeApi::send() { udp_.Send(); }

void UnitreeApi::recv() { udp_.Recv(); }

constexpr const char *UnitreeApi::getRobotName() {
#if STEPIT_UNITREE_ROBOT == UNITREE_ALIENGO
  return "aliengo";
#elif STEPIT_UNITREE_ROBOT == UNITREE_GO1
  return "go1";
#elif STEPIT_UNITREE_ROBOT == UNITREE_B1
  return "b1";
#else
  return "";
#endif
}

#if STEPIT_UNITREE_ROBOT == UNITREE_ALIENGO
STEPIT_REGISTER_ROBOTAPI(aliengo, kDefPriority, RobotApi::make<UnitreeApi>);
#elif STEPIT_UNITREE_ROBOT == UNITREE_GO1
STEPIT_REGISTER_ROBOTAPI(go1, kDefPriority, RobotApi::make<UnitreeApi>);
#elif STEPIT_UNITREE_ROBOT == UNITREE_B1
STEPIT_REGISTER_ROBOTAPI(b1, kDefPriority, RobotApi::make<UnitreeApi>);
#else
#error "Unsupported robot type."
#endif  // STEPIT_UNITREE_ROBOT
}  // namespace stepit

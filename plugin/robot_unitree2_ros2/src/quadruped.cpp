#include <array>
#include <cstddef>
#include <cstdint>

#include <stepit/logging.h>

#include <stepit/robot/unitree2/go2.h>
#include <stepit/robot/unitree2_ros2/quadruped.h>

namespace stepit {
Unitree2Ros2Quadruped::Unitree2Ros2Quadruped(const std::string &robot_name)
    : RobotApi(robot_name), low_state_(getDoF(), getNumLegs()) {
  low_cmd_.head[0]    = 0xFE;
  low_cmd_.head[1]    = 0xEF;
  low_cmd_.level_flag = 0xFF;
  low_cmd_.gpio       = 0;
  for (auto &motor_cmd : low_cmd_.motor_cmd) {
    motor_cmd.mode = robot_name == "b2" ? kB2MotorServoMode : kGo2MotorServoMode;
    motor_cmd.q    = kPosStop;
    motor_cmd.dq   = kVelStop;
    motor_cmd.kp   = 0;
    motor_cmd.kd   = 0;
    motor_cmd.tau  = 0;
  }
}

void Unitree2Ros2Quadruped::getControl(bool enable) {
  if (enable) {
    low_cmd_pub_   = getNode()->create_publisher<u2ros2_msg::LowCmd>("/lowcmd", 1);
    low_state_sub_ = getNode()->create_subscription<u2ros2_msg::LowState>(
        "/lowstate", 1, [this](const u2ros2_msg::LowState::SharedPtr msg) { callback(msg); });
  }
}

void Unitree2Ros2Quadruped::setSend(const LowCmd &cmd_msg) {
  for (std::size_t i{}; i < getDoF(); ++i) {
    low_cmd_.motor_cmd[i].q   = cmd_msg[i].q;
    low_cmd_.motor_cmd[i].dq  = cmd_msg[i].dq;
    low_cmd_.motor_cmd[i].kp  = cmd_msg[i].Kp;
    low_cmd_.motor_cmd[i].kd  = cmd_msg[i].Kd;
    low_cmd_.motor_cmd[i].tau = cmd_msg[i].tor;
  }
}

void Unitree2Ros2Quadruped::getRecv(LowState &state_msg) {
  std::lock_guard<std::mutex> _(mutex_);
  state_msg = low_state_;
}

void toU2LowCmd(u2_msg::LowCmd_ &u2, const u2ros2_msg::LowCmd &u2r2) {
  u2.head()          = u2r2.head;
  u2.level_flag()    = u2r2.level_flag;
  u2.frame_reserve() = u2r2.frame_reserve;
  u2.sn()            = u2r2.sn;
  u2.version()       = u2r2.version;
  u2.bandwidth()     = u2r2.bandwidth;

  for (std::size_t i{}; i < u2.motor_cmd().size(); i++) {
    u2.motor_cmd()[i].mode()    = u2r2.motor_cmd[i].mode;
    u2.motor_cmd()[i].q()       = u2r2.motor_cmd[i].q;
    u2.motor_cmd()[i].dq()      = u2r2.motor_cmd[i].dq;
    u2.motor_cmd()[i].tau()     = u2r2.motor_cmd[i].tau;
    u2.motor_cmd()[i].kp()      = u2r2.motor_cmd[i].kp;
    u2.motor_cmd()[i].kd()      = u2r2.motor_cmd[i].kd;
    u2.motor_cmd()[i].reserve() = u2r2.motor_cmd[i].reserve;
  }

  u2.bms_cmd().off()     = u2r2.bms_cmd.off;
  u2.bms_cmd().reserve() = u2r2.bms_cmd.reserve;
  u2.wireless_remote()   = u2r2.wireless_remote;
  u2.led()               = u2r2.led;
  u2.fan()               = u2r2.fan;
  u2.gpio()              = u2r2.gpio;
  u2.reserve()           = u2r2.reserve;
}

void Unitree2Ros2Quadruped::send() {
  u2_msg::LowCmd_ u2cmd;
  toU2LowCmd(u2cmd, low_cmd_);
  low_cmd_.crc = crc32core(reinterpret_cast<uint32_t *>(&u2cmd), (sizeof(u2_msg::LowCmd_) >> 2) - 1);
  low_cmd_pub_->publish(low_cmd_);
}

void Unitree2Ros2Quadruped::callback(const u2ros2_msg::LowState::SharedPtr msg) {
  std::lock_guard<std::mutex> _(mutex_);
  low_state_.imu.rpy           = msg->imu_state.rpy;
  low_state_.imu.quaternion    = msg->imu_state.quaternion;
  low_state_.imu.accelerometer = msg->imu_state.accelerometer;
  low_state_.imu.gyroscope     = msg->imu_state.gyroscope;

  for (std::size_t i{}; i < getDoF(); ++i) {
    low_state_.motor_state[i].q   = msg->motor_state[i].q;
    low_state_.motor_state[i].dq  = msg->motor_state[i].dq;
    low_state_.motor_state[i].tor = msg->motor_state[i].tau_est;
  }
  for (std::size_t i{}; i < getNumLegs(); ++i) {
    low_state_.foot_force[i] = msg->foot_force[i];
  }
  low_state_.tick = msg->tick;
}

STEPIT_REGISTER_ROBOTAPI(b2_ros2, kDefPriority, [] { return std::make_unique<Unitree2Ros2Quadruped>("b2"); });
STEPIT_REGISTER_ROBOTAPI(go2_ros2, kDefPriority, [] { return std::make_unique<Unitree2Ros2Quadruped>("go2"); });
}  // namespace stepit

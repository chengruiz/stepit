#include <stepit/robot/unitree2/g1.h>

namespace stepit {
G1Api::G1Api() : RobotApi(kRobotName), low_state_(getDoF(), getNumLegs()) {
  low_cmd_.mode_pr(0);
  if (yml::hasValue(config_, "motor_enabled")) {
    yml::setTo(config_, "motor_enabled", motor_enabled_);
  } else {
    motor_enabled_.fill(true);
  }

  for (int i{}; i < getDoF(); ++i) {
    low_cmd_.motor_cmd()[i].mode() = motor_enabled_[i] ? 1 : 0;
    low_cmd_.motor_cmd()[i].q()    = kPosStop;
    low_cmd_.motor_cmd()[i].dq()   = kVelStop;
    low_cmd_.motor_cmd()[i].tau()  = 0;
    low_cmd_.motor_cmd()[i].kp()   = 0;
    low_cmd_.motor_cmd()[i].kd()   = 0;
  }
}

void G1Api::getControl(bool enable) {
  if (enable) {
    Unitree2ServiceClient::initialize();
    low_cmd_pub_   = std::make_shared<u2_sdk::ChannelPublisher<hg_msg::LowCmd_>>("rt/lowcmd");
    low_state_sub_ = std::make_shared<u2_sdk::ChannelSubscriber<hg_msg::LowState_>>("rt/lowstate");
    low_cmd_pub_->InitChannel();
    low_state_sub_->InitChannel([this](const void *msg) { callback(static_cast<const hg_msg::LowState_ *>(msg)); }, 1);
  }
}

void G1Api::setSend(const LowCmd &cmd_msg) {
  low_cmd_.mode_machine(mode_machine_.load(std::memory_order_relaxed));
  for (int i{}; i < getDoF(); ++i) {
    low_cmd_.motor_cmd()[i].mode() = motor_enabled_[i] ? 1 : 0;
    low_cmd_.motor_cmd()[i].q()    = cmd_msg[i].q;
    low_cmd_.motor_cmd()[i].dq()   = cmd_msg[i].dq;
    low_cmd_.motor_cmd()[i].kp()   = cmd_msg[i].Kp;
    low_cmd_.motor_cmd()[i].kd()   = cmd_msg[i].Kd;
    low_cmd_.motor_cmd()[i].tau()  = cmd_msg[i].tor;
  }
}

void G1Api::getRecv(LowState &state_msg) {
  std::lock_guard<std::mutex> _(mutex_);
  state_msg = low_state_;
}

void G1Api::send() {
  fillLowCmdCrc(low_cmd_);
  low_cmd_pub_->Write(low_cmd_);
}

void G1Api::callback(const hg_msg::LowState_ *msg) {
  std::lock_guard<std::mutex> _(mutex_);
  mode_machine_.store(msg->mode_machine(), std::memory_order_relaxed);
  low_state_.imu.rpy           = msg->imu_state().rpy();
  low_state_.imu.quaternion    = msg->imu_state().quaternion();
  low_state_.imu.accelerometer = msg->imu_state().accelerometer();
  low_state_.imu.gyroscope     = msg->imu_state().gyroscope();

  for (int i{}; i < getDoF(); ++i) {
    low_state_.motor_state[i].q   = msg->motor_state()[i].q();
    low_state_.motor_state[i].dq  = msg->motor_state()[i].dq();
    low_state_.motor_state[i].tor = msg->motor_state()[i].tau_est();
  }
  low_state_.tick = msg->tick();
}

const std::array<std::size_t, 29> kJointOrderG1Dfs2Bfs = {
    0,   // left_hip_pitch_joint
    6,   // right_hip_pitch_joint
    12,  // waist_yaw_joint
    1,   // left_hip_roll_joint
    7,   // right_hip_roll_joint
    13,  // waist_roll_joint
    2,   // left_hip_yaw_joint
    8,   // right_hip_yaw_joint
    14,  // waist_pitch_joint
    3,   // left_knee_joint
    9,   // right_knee_joint
    15,  // left_shoulder_pitch_joint
    22,  // right_shoulder_pitch_joint
    4,   // left_ankle_pitch_joint
    10,  // right_ankle_pitch_joint
    16,  // left_shoulder_roll_joint
    23,  // right_shoulder_roll_joint
    5,   // left_ankle_roll_joint
    11,  // right_ankle_roll_joint
    17,  // left_shoulder_yaw_joint
    24,  // right_shoulder_yaw_joint
    18,  // left_elbow_joint
    25,  // right_elbow_joint
    19,  // left_wrist_roll_joint
    26,  // right_wrist_roll_joint
    20,  // left_wrist_pitch_joint
    27,  // right_wrist_pitch_joint
    21,  // left_wrist_yaw_joint
    28,  // right_wrist_yaw_joint
};

STEPIT_REGISTER_ROBOTAPI(g1, kDefPriority, RobotApi::make<G1Api>);
STEPIT_REGISTER_ROBOTAPI(g1_bfs, kDefPriority, []() {
  return std::make_unique<RobotApiReorderingWrapper>("g1_bfs", "g1", array2vector(kJointOrderG1Dfs2Bfs));
});
}  // namespace stepit

#include <stepit/publisher_csv/publisher_csv.h>

namespace stepit {
CsvPublisher::CsvPublisher() {
  std::string filename = fmt::format("low_level_{}.csv", getCurrentTimeStamp("%Y%m%d_%H%M%S", false));
  file_.open(filename);
  STEPIT_ASSERT(file_, "Failed to open file {}.", filename);
}

CsvPublisher::~CsvPublisher() { file_.close(); }

void CsvPublisher::publishLowLevel(const RobotSpec &spec, const LowState &state, const LowCmd &cmd) {
  if (not header_written_) {
    file_ << "tick"
             ",imu_w,imu_x,imu_y,imu_z"
             ",gyro_x,gyro_y,gyro_z"
             ",acc_x,acc_y,acc_z"
             ",roll,pitch,yaw";
    for (std::size_t i{}; i < state.foot_force.size(); ++i) {
      file_ << ",foot_" << i;
    }
    for (std::size_t i{}; i < state.motor_state.size(); ++i) {
      file_ << ",q_" << i << ",dq_" << i << ",tor_" << i;
    }
    for (std::size_t i{}; i < cmd.size(); ++i) {
      file_ << ",cmd_q_" << i << ",cmd_dq_" << i << ",cmd_tor_" << i << ",cmd_Kp_" << i << ",cmd_Kd_" << i;
    }
    file_ << '\n';
    header_written_ = true;
  }

  const auto &imu = state.imu;
  file_ << state.tick
        << fmt::format(",{},{},{},{}", imu.quaternion[0], imu.quaternion[1], imu.quaternion[2], imu.quaternion[3])
        << fmt::format(",{},{},{}", imu.gyroscope[0], imu.gyroscope[1], imu.gyroscope[2])
        << fmt::format(",{},{},{}", imu.accelerometer[0], imu.accelerometer[1], imu.accelerometer[2])
        << fmt::format(",{},{},{}", imu.rpy[0], imu.rpy[1], imu.rpy[2]);

  for (float foot_force : state.foot_force) file_ << "," << foot_force;
  for (const auto &joint_state : state.motor_state) {
    file_ << fmt::format(",{},{},{}", joint_state.q, joint_state.dq, joint_state.tor);
  }
  for (const auto &joint_cmd : cmd) {
    file_ << fmt::format(",{},{},{},{},{}", joint_cmd.q, joint_cmd.dq, joint_cmd.tor, joint_cmd.Kp, joint_cmd.Kd);
  }
  file_ << '\n';
}

STEPIT_REGISTER_PUBLISHER(csv, kDefPriority, Publisher::make<CsvPublisher>);
}  // namespace stepit

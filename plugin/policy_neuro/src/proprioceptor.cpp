#include <stepit/policy_neuro/proprioceptor.h>

namespace stepit {
namespace neuro_policy {
Proprioceptor::Proprioceptor(const PolicySpec &policy_spec, const std::string &) {
  ang_vel_id_   = registerProvision("ang_vel", 3);
  gravity_id_   = registerProvision("gravity", 3);
  joint_pos_id_ = registerProvision("joint_pos", policy_spec.dof);
  joint_vel_id_ = registerProvision("joint_vel", policy_spec.dof);
  lin_acc_id_   = registerProvision("lin_acc", 3);
}

bool Proprioceptor::update(const LowState &low_state, ControlRequests &requests, FieldMap &context) {
  std::uint32_t num_joints = getFieldSize(joint_pos_id_);
  ArrXf joint_pos{num_joints}, joint_vel{num_joints};
  for (std::uint32_t i{}; i < num_joints; ++i) {
    joint_pos[i] = low_state.motor_state[i].q;
    joint_vel[i] = low_state.motor_state[i].dq;
  }

  context[lin_acc_id_]   = cmArr3f(low_state.imu.accelerometer.data());
  context[ang_vel_id_]   = cmArr3f(low_state.imu.gyroscope.data());
  context[gravity_id_]   = Quatf(low_state.imu.quaternion).inverse() * Vec3f{0, 0, -1.};
  context[joint_pos_id_] = joint_pos;
  context[joint_vel_id_] = joint_vel;
  return true;
}

RollPitchSource::RollPitchSource(const PolicySpec &, const std::string &) {
  roll_pitch_id_ = registerProvision("roll_pitch", 2);
}

bool RollPitchSource::update(const LowState &low_state, ControlRequests &requests, FieldMap &context) {
  context[roll_pitch_id_] = Arr2f{low_state.imu.rpy[0], low_state.imu.rpy[1]};
  return true;
}

STEPIT_REGISTER_MODULE(proprioceptor, kDefPriority, Module::make<Proprioceptor>);
STEPIT_REGISTER_MODULE(roll_pitch, kDefPriority, Module::make<RollPitchSource>);
STEPIT_REGISTER_FIELD_SOURCE(ang_vel, kDefPriority, Module::make<Proprioceptor>);
STEPIT_REGISTER_FIELD_SOURCE(gravity, kDefPriority, Module::make<Proprioceptor>);
STEPIT_REGISTER_FIELD_SOURCE(joint_pos, kDefPriority, Module::make<Proprioceptor>);
STEPIT_REGISTER_FIELD_SOURCE(joint_vel, kDefPriority, Module::make<Proprioceptor>);
STEPIT_REGISTER_FIELD_SOURCE(lin_acc, kDefPriority, Module::make<Proprioceptor>);
STEPIT_REGISTER_FIELD_SOURCE(roll_pitch, kDefPriority, Module::make<RollPitchSource>);
}  // namespace neuro_policy
}  // namespace stepit

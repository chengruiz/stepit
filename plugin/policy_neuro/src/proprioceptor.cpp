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

bool Proprioceptor::update(const LowState &low_state, ControlRequests &requests, FieldMap &result) {
  std::size_t joint_dim{getFieldSize(joint_pos_id_)};
  ArrXf joint_pos{joint_dim}, joint_vel{joint_dim};
  for (std::size_t i{}; i < joint_dim; ++i) {
    joint_pos[i] = low_state.motor_state[i].q;
    joint_vel[i] = low_state.motor_state[i].dq;
  }

  result[lin_acc_id_]   = cmArr3f(low_state.imu.accelerometer.data());
  result[ang_vel_id_]   = cmArr3f(low_state.imu.gyroscope.data());
  result[gravity_id_]   = Quatf(low_state.imu.quaternion).inverse() * Vec3f{0, 0, -1.};
  result[joint_pos_id_] = joint_pos;
  result[joint_vel_id_] = joint_vel;
  return true;
}

STEPIT_REGISTER_MODULE(proprioceptor, kDefPriority, Module::make<Proprioceptor>);
STEPIT_REGISTER_FIELD_SOURCE(ang_vel, kDefPriority, Module::make<Proprioceptor>);
STEPIT_REGISTER_FIELD_SOURCE(gravity, kDefPriority, Module::make<Proprioceptor>);
STEPIT_REGISTER_FIELD_SOURCE(joint_pos, kDefPriority, Module::make<Proprioceptor>);
STEPIT_REGISTER_FIELD_SOURCE(joint_vel, kDefPriority, Module::make<Proprioceptor>);
STEPIT_REGISTER_FIELD_SOURCE(lin_acc, kDefPriority, Module::make<Proprioceptor>);
}  // namespace neuro_policy
}  // namespace stepit

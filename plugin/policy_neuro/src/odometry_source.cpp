#include <stepit/policy_neuro/odometry_source.h>

namespace stepit {
namespace neuro_policy {
DummyOdometrySource::DummyOdometrySource(const NeuroPolicySpec &policy_spec, const std::string &name)
    : Module(policy_spec, nonEmptyOr(name, "odometry"), true) {
  base_global_pos_id_ = registerProvision("base_global_pos", 3);
  base_global_ori_id_ = registerProvision("base_global_ori", 4);
}

bool DummyOdometrySource::reset() {
  world_frame_.setIdentity();
  initialized_ = false;
  return true;
}

bool DummyOdometrySource::update(const LowState &low_state, ControlRequests &, FieldMap &context) {
  if (not initialized_) {
    world_frame_ = Quatf::fromYaw(low_state.imu.rpy[2]);
    initialized_ = true;
  }

  Quatf current_ori(low_state.imu.quaternion);
  context[base_global_pos_id_] = Vec3f::Zero();
  Quatf global_ori             = world_frame_.inverse() * current_ori;
  context[base_global_ori_id_] = global_ori.coeffs();
  return true;
}

STEPIT_REGISTER_MODULE(odometry, kMinPriority, Module::make<DummyOdometrySource>);
STEPIT_REGISTER_FIELD_SOURCE(base_global_pos, kMinPriority, Module::make<DummyOdometrySource>);
STEPIT_REGISTER_FIELD_SOURCE(base_global_ori, kMinPriority, Module::make<DummyOdometrySource>);
}  // namespace neuro_policy
}  // namespace stepit

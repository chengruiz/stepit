#ifndef STEPIT_NEURO_POLICY_ACTUATOR_H_
#define STEPIT_NEURO_POLICY_ACTUATOR_H_

#include <stepit/policy_neuro/field.h>

namespace stepit {
namespace neuro_policy {
class Actuator : public Module,
                 public Interface<Actuator, const PolicySpec & /* policy_spec */, const std::string & /* home_dir */> {
 public:
  using Interface    = Interface<Actuator, const PolicySpec &, const std::string &>;
  using Ptr          = Interface::Ptr;
  using Registry     = Interface::Registry;
  using Registration = Interface::Registration;
  using Factory      = Interface::Factory;
  using Interface::make;

  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &result) override { return true; }
  virtual void setLowCmd(LowCmd &cmd, cArrXf action) = 0;
};

class PositionActuator : public Actuator {
 public:
  PositionActuator(const PolicySpec &policy_spec, const std::string &home_dir);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &result) override;
  void setLowCmd(LowCmd &cmd, cArrXf action) override;

 private:
  YAML::Node config_;
  ArrXf scale_, bias_;
  ArrXf kp_, kd_;
  FieldId last_target_joint_pos_id_;

  bool is_first_update_{false};
  ArrXf target_joint_pos_;
};

class VelocityActuator : public Actuator {
 public:
  VelocityActuator(const PolicySpec &policy_spec, const std::string &home_dir);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &result) override;
  void setLowCmd(LowCmd &cmd, cArrXf action) override;

 private:
  YAML::Node config_;
  ArrXf scale_, bias_;
  ArrXf kp_, kd_;
  FieldId last_target_joint_vel_id_;

  ArrXf target_joint_vel_;
};

class TorqueActuator : public Actuator {
 public:
  TorqueActuator(const PolicySpec &policy_spec, const std::string &home_dir);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &result) override;
  void setLowCmd(LowCmd &cmd, cArrXf action) override;

 private:
  YAML::Node config_;
  ArrXf scale_, bias_;
  ArrXf kp_, kd_;
  FieldId last_target_joint_tor_id_;

  ArrXf target_joint_tor_;
};

class HybridActuator : public Actuator {
 public:
  HybridActuator(const PolicySpec &policy_spec, const std::string &home_dir);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &result) override;
  void setLowCmd(LowCmd &cmd, cArrXf action) override;

  enum class Mode { kPosition, kVelocity, kTorque };

 private:
  YAML::Node config_;
  ArrXf scale_, bias_;
  ArrXf kp_, kd_;
  std::vector<Mode> modes_;
  FieldId last_joint_command_id_;

  ArrXf joint_command_;
};
}  // namespace neuro_policy
}  // namespace stepit

#define STEPIT_REGISTER_ACTUATOR(name, priority, factory) \
  static ::stepit::neuro_policy::Actuator::Registration _actuator_##name##_registration(#name, priority, factory)

#endif  // STEPIT_NEURO_POLICY_ACTUATOR_H_

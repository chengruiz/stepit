#ifndef STEPIT_NEURO_POLICY_ACTUATOR_H_
#define STEPIT_NEURO_POLICY_ACTUATOR_H_

#include <stepit/policy_neuro/field.h>

namespace stepit {
namespace neuro_policy {
class Actuator : public Module,
                 public Interface<Actuator, const NeuroPolicySpec & /* policy_spec */, const std::string & /* name */> {
 public:
  using Interface    = Interface<Actuator, const NeuroPolicySpec &, const std::string &>;
  using Ptr          = Interface::Ptr;
  using Registry     = Interface::Registry;
  using Registration = Interface::Registration;
  using Factory      = Interface::Factory;
  using Interface::make;

  Actuator(const NeuroPolicySpec &policy_spec, const std::string &name);
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override { return true; }
  virtual void setLowCmd(LowCmd &cmd, cArrXf action) = 0;

 protected:
  ArrXf scale_, bias_;
  ArrXf kp_, kd_;
};

class PositionActuator : public Actuator {
 public:
  PositionActuator(const NeuroPolicySpec &policy_spec, const std::string &name);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;
  void setLowCmd(LowCmd &cmd, cArrXf action) override;

 private:
  FieldId last_target_joint_pos_id_;
  bool is_first_update_{false};
  ArrXf target_joint_pos_;
};

class VelocityActuator : public Actuator {
 public:
  VelocityActuator(const NeuroPolicySpec &policy_spec, const std::string &name);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;
  void setLowCmd(LowCmd &cmd, cArrXf action) override;

 private:
  FieldId last_target_joint_vel_id_;
  ArrXf target_joint_vel_;
};

class TorqueActuator : public Actuator {
 public:
  TorqueActuator(const NeuroPolicySpec &policy_spec, const std::string &name);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;
  void setLowCmd(LowCmd &cmd, cArrXf action) override;

 private:
  FieldId last_target_joint_tor_id_;
  ArrXf target_joint_tor_;
};

class HybridActuator : public Actuator {
 public:
  HybridActuator(const NeuroPolicySpec &policy_spec, const std::string &name);
  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;
  void setLowCmd(LowCmd &cmd, cArrXf action) override;

 private:
  enum class Mode { kPosition, kVelocity, kTorque };
  static const std::map<std::string, Mode> kModeMap;

  std::vector<Mode> modes_;
  FieldId last_joint_command_id_;
  ArrXf joint_command_;
};
}  // namespace neuro_policy
}  // namespace stepit

#define STEPIT_REGISTER_ACTUATOR(name, priority, factory) \
  static ::stepit::neuro_policy::Actuator::Registration _actuator_##name##_registration(#name, priority, factory)

#endif  // STEPIT_NEURO_POLICY_ACTUATOR_H_

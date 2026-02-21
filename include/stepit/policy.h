#ifndef STEPIT_POLICY_H_
#define STEPIT_POLICY_H_

#include <stepit/communication.h>
#include <stepit/control_input.h>
#include <stepit/robot.h>

namespace stepit {
struct PolicySpec : RobotSpec {
  explicit PolicySpec(const RobotSpec &robot_spec, std::string home_dir)
      : RobotSpec(robot_spec), home_dir(std::move(home_dir)) {}

  /* Home directory for the policy to read configuration files. */
  std::string home_dir;
  /* Name of the policy */
  std::string policy_name;
  /* Control frequency in Hz */
  std::size_t control_freq{};
  /* Whether the policy is trusted in the case of safety violations */
  bool trusted{};
};

class Policy : public Interface<Policy, const RobotSpec & /* robot_spec */, const std::string & /* home_dir */> {
 public:
  virtual const PolicySpec &getSpec() const = 0;
  std::string getHomeDir() const { return getSpec().home_dir; }
  std::size_t getControlFreq() const { return getSpec().control_freq; }
  std::string getName() const { return getSpec().policy_name; }
  bool isTrusted() const { return getSpec().trusted; }

  /**
   * @brief Resets the policy to its initial state.
   *
   * @return True if reset succeeded; otherwise, false.
   */
  virtual bool reset() = 0;
  /**
   * @brief Produce a control action based on the current low-level state.
   *
   * @param low_state Current low-level state information to base the control decision on.
   * @param requests Current control requests to be replied by the policy.
   * @param cmd Output parameter to be populated with the resulting low-level command.
   * @return True if the action was successfully computed; otherwise false.
   */
  virtual bool act(const LowState &low_state, ControlRequests &requests, LowCmd &cmd) = 0;
  /**
   * @brief Shuts down the policy, releasing resources and performing any necessary finalization.
   */
  virtual void exit() = 0;
};
}  // namespace stepit

#define STEPIT_REGISTER_POLICY(name, priority, factory) \
  static ::stepit::Policy::Registration _policy_##name##_registration(#name, priority, factory)

#endif  // STEPIT_POLICY_H_

#ifndef STEPIT_JOYSTICK_H_
#define STEPIT_JOYSTICK_H_

#include <map>
#include <mutex>

#include <boost/optional.hpp>

#include <stepit/control_input.h>
#include <stepit/registry.h>
#include <stepit/joystick/state.h>

namespace stepit {
namespace joystick {
class Joystick : public Interface<Joystick> {
 public:
  virtual bool connected() const = 0;
  virtual void getState(State &) = 0;
};

class JoystickControl : public ControlInput {
 public:
  explicit JoystickControl(const std::string &js_type = "");
  bool available() const override { return js_->connected(); }
  void poll() override;

  class Registration;
  using Rule = std::function<boost::optional<std::string>(const State &)>;

 protected:
  Joystick::Ptr js_;
  State state_;

  static std::map<std::size_t, Rule> rules_;
  static std::size_t rule_count_;
  static std::mutex rule_mutex_;
};

class JoystickControl::Registration {
 public:
  explicit Registration(Rule rule);
  ~Registration();
  Registration(const Registration &)            = delete;
  Registration &operator=(const Registration &) = delete;
  Registration(Registration &&) noexcept;
  Registration &operator=(Registration &&) noexcept;

 private:
  std::size_t rule_id_;
};
}  // namespace joystick

using joystick::Joystick;
using joystick::JoystickControl;
}  // namespace stepit

#define STEPIT_REGISTER_JOYSTICK(name, priority, factory) \
  static ::stepit::Joystick::Registration _joystick_##name##_registration(#name, priority, factory)

#endif  // STEPIT_JOYSTICK_H_

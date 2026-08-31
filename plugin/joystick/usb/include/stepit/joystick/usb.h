#ifndef STEPIT_USB_JOYSTICK_H_
#define STEPIT_USB_JOYSTICK_H_

#include <atomic>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <linux/joystick.h>

#include <stepit/joystick/joystick.h>
#include <stepit/joystick/keymap.h>

namespace stepit {
namespace joystick {
class UsbJoystick final : public Joystick {
 public:
  explicit UsbJoystick();
  ~UsbJoystick() override;
  bool connected() const override { return connected_.load(); }
  void getState(State &state) override;

 private:
  struct DeviceInfo {
    long id{-1};
    std::string path;
  };

  void run();
  bool tryConnect(const DeviceInfo &device);
  std::vector<DeviceInfo> listDevices() const;
  bool loadKeymapByName(const std::string &name, Keymap &keymap) const;
  void disconnect(bool log_disconnect);
  void processEvent(const struct js_event &event);
  void updateAxis(std::size_t aid, float value, bool emit_transient);
  void updateButton(std::size_t bid, bool pressed, bool emit_transient);

  long id_{-1};
  int fd_{-1};
  std::string device_path_;
  std::string device_name_;

  std::mutex mutex_;
  Slots slots_;
  Keymap keymap_;
  std::set<std::string> ignored_devices_;
  std::atomic<bool> connected_{false};
  std::atomic<bool> running_{false};
  std::thread worker_;
};
}  // namespace joystick
}  // namespace stepit

#endif  // STEPIT_USB_JOYSTICK_H_

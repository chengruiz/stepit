#include <cstdio>

#include <stepit/utils.h>
#include <stepit/joystick/usb.h>

namespace stepit {
namespace joystick {
static void libGamepadLogger(int level, const char *format, va_list args, void *) {
  char out[4096];
  std::vsnprintf(out, sizeof(out), format, args);
  switch (level) {
    case gamepad::LOG_INFO:
      STEPIT_LOG("Gamepad: {}", out);
      break;
    case gamepad::LOG_WARNING:
      if (std::string(out).find("is still in use! (Ref count 2)") != std::string::npos) return;
      STEPIT_WARN("Gamepad: {}", out);
      break;
    case gamepad::LOG_ERROR:
      STEPIT_CRIT("Gamepad: {}", out);
      break;
    default:
      break;
  }
}

void initializeLibGamepad() {
  static bool initialized = false;
  if (initialized) return;
  gamepad::set_logger(libGamepadLogger, nullptr);
  initialized = true;
}

UsbJoystick::UsbJoystick() {
  initializeLibGamepad();

  getenv("STEPIT_JOYSTICK_ID", id_);
  hook_ = gamepad::hook::make();
  hook_->set_plug_and_play(true, gamepad::ms(1000));
  hook_->set_sleep_time(gamepad::ms(1));

  hook_->set_connect_event_handler([this](auto dev) { connectHandler(std::move(dev)); });
  hook_->set_axis_event_handler([this](auto dev) { axisHandler(std::move(dev)); });
  hook_->set_button_event_handler([this](auto dev) { buttonHandler(std::move(dev)); });
  hook_->set_disconnect_event_handler([this](auto dev) { disconnectHandler(std::move(dev)); });
  STEPIT_ASSERT(hook_->start(), "Failed to start gamepad hook.");
}

UsbJoystick::~UsbJoystick() {
  if (hook_) hook_->stop();
}

void UsbJoystick::getState(State &state) {
  if (not connected_) {
    state = {};
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  populateStateFromSlots(keymap_, slots_, state);

  for (auto &button : slots_.buttons) button.resetTransientStates();
}

void UsbJoystick::connectHandler(std::shared_ptr<gamepad::device> dev) {
  if (connected_ or (id_ >= 0 and dev->get_index() != id_)) return;

  std::string name = toLowercase(dev->get_name());
  if (name.find("x-box") != std::string::npos or name.find("xbox") != std::string::npos) {
    keymap_ = getXboxKeymap();
  } else {
    bool found = false;
    // Search for the keymap file in the config directory
    for (const auto &entry : fs::directory_iterator(getConfigPath("joystick"))) {
      if (entry.path().extension() != ".yml") continue;
      std::string stem = toLowercase(entry.path().stem().string());
      if (name.find(stem) != std::string::npos) {
        keymap_ = Keymap{yml::loadFile(entry.path().string())};
        found   = true;
        break;
      }
    }
    if (not found) {
      STEPIT_WARN("An unsupported joystick '{}' connected. Ignored.", dev->get_name());
      return;
    }
  }

  STEPIT_INFO("Joystick '{}' connected.", dev->get_name());
  connected_ = true;
  rid_       = dev->get_index();
  slots_.axes[keymap_.lt] = -1.0;
  slots_.axes[keymap_.rt] = -1.0;
  dev->set_axis_deadzone(gamepad::axis::LEFT_STICK_X, 0);
  dev->set_axis_deadzone(gamepad::axis::LEFT_STICK_Y, 0);
  dev->set_axis_deadzone(gamepad::axis::LEFT_TRIGGER, 0);
  dev->set_axis_deadzone(gamepad::axis::RIGHT_STICK_X, 0);
  dev->set_axis_deadzone(gamepad::axis::RIGHT_STICK_Y, 0);
  dev->set_axis_deadzone(gamepad::axis::RIGHT_TRIGGER, 0);
}

void UsbJoystick::disconnectHandler(std::shared_ptr<gamepad::device> dev) {
  if (rid_ != dev->get_index()) return;
  connected_ = false;
  rid_       = -1;
  STEPIT_WARN("Joystick '{}' disconnected.", dev->get_name());
}

void UsbJoystick::axisHandler(std::shared_ptr<gamepad::device> dev) {
  if (rid_ != dev->get_index()) return;
  uint16_t aid = dev->last_axis_event()->native_id;
  if (aid >= Slots::NAxes) return;

  float value = dev->last_axis_event()->virtual_value;
  std::lock_guard<std::mutex> lock(mutex_);
  // map from [0, 1] to [-1, 1]
  slots_.axes[aid] = value * 2 - 1;
  // Convert the analogue axis to two virtual buttons, one for each direction
  slots_.buttons[Slots::NButtons + 2 * aid].update(value < 0.01);
  slots_.buttons[Slots::NButtons + 2 * aid + 1].update(value > 0.99);
}

void UsbJoystick::buttonHandler(std::shared_ptr<gamepad::device> dev) {
  if (rid_ != dev->get_index()) return;
  uint16_t bid = dev->last_button_event()->native_id;
  if (bid >= Slots::NButtons) return;

  bool pressed = dev->last_button_event()->virtual_value > 0.99;
  std::lock_guard<std::mutex> lock(mutex_);
  slots_.buttons[bid].update(pressed);
}

STEPIT_REGISTER_JOYSTICK(usb, kDefPriority, Joystick::make<UsbJoystick>);
}  // namespace joystick
}  // namespace stepit

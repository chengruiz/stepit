#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdlib>

#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <sys/ioctl.h>

#include <stepit/utils.h>
#include <stepit/joystick/usb.h>

namespace stepit {
namespace joystick {
namespace {
constexpr auto kPollInterval          = std::chrono::milliseconds(1);
constexpr auto kScanInterval          = std::chrono::seconds(1);
constexpr const char *kInputDirectory = "/dev/input";

bool parseJoystickId(const std::string &name, long &id) {
  if (name.size() <= 2 or name[0] != 'j' or name[1] != 's') return false;

  char *end         = nullptr;
  const char *begin = name.c_str() + 2;
  long value        = std::strtol(begin, &end, 10);
  if (begin == end or *end != '\0' or value < 0) return false;

  id = value;
  return true;
}

float normalizeAxisValue(int16_t value) {
  return std::max(-1.0f, std::min(1.0f, static_cast<float>(value) / 32767.0f));
}

void setButtonState(Button &button, bool pressed, bool emit_transient) {
  if (emit_transient) {
    button.update(pressed);
  } else {
    button.pressed = pressed;
  }
}
}  // namespace

UsbJoystick::UsbJoystick() {
  if (not getenv("STEPIT_JS_ID", id_)) getenv("STEPIT_JOYSTICK_ID", id_);

  running_ = true;
  worker_  = std::thread(&UsbJoystick::run, this);
}

UsbJoystick::~UsbJoystick() {
  running_ = false;
  if (worker_.joinable()) worker_.join();
  disconnect(false);
}

void UsbJoystick::run() {
  while (running_) {
    if (not connected_) {
      for (const auto &device : listDevices()) {
        if (not running_ or connected_) break;
        if (id_ >= 0 and device.id != id_) continue;
        if (tryConnect(device)) break;
      }

      auto slept = std::chrono::milliseconds(0);
      while (running_ and not connected_ and slept < kScanInterval) {
        std::this_thread::sleep_for(kPollInterval);
        slept += kPollInterval;
      }
      continue;
    }

    js_event event{};
    bool disconnected = false;
    while (running_) {
      ssize_t result = ::read(fd_, &event, sizeof(event));
      if (result == static_cast<ssize_t>(sizeof(event))) {
        processEvent(event);
        continue;
      }

      if (result < 0 and errno == EINTR) continue;
      if (result < 0 and (errno == EAGAIN or errno == EWOULDBLOCK)) break;

      disconnected = true;
      break;
    }

    if (not disconnected and connected_) {
      boost::system::error_code ec;
      if (not device_path_.empty() and not fs::exists(device_path_, ec) and not ec) disconnected = true;
    }

    if (disconnected) disconnect(true);
    std::this_thread::sleep_for(kPollInterval);
  }
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

bool UsbJoystick::tryConnect(const DeviceInfo &device) {
  int fd = ::open(device.path.c_str(), O_RDONLY | O_NONBLOCK);
  if (fd < 0) return false;

  char name_buffer[256] = {};
  std::string name;
  if (::ioctl(fd, JSIOCGNAME(sizeof(name_buffer)), name_buffer) >= 0 and name_buffer[0] != '\0') {
    name = name_buffer;
  } else {
    name = fs::path(device.path).filename().string();
  }

  Keymap keymap;
  if (not loadKeymapByName(name, keymap)) {
    if (ignored_devices_.insert(device.path).second) {
      STEPIT_WARN("An unsupported joystick '{}' connected. Ignored.", name);
    }
    ::close(fd);
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    slots_                  = {};
    keymap_                 = keymap;
    slots_.axes[keymap_.lt] = -1.0f;
    slots_.axes[keymap_.rt] = -1.0f;
  }

  fd_          = fd;
  device_path_ = device.path;
  device_name_ = name;
  connected_   = true;

  STEPIT_INFO("Joystick '{}' connected.", device_name_);
  return true;
}

std::vector<UsbJoystick::DeviceInfo> UsbJoystick::listDevices() const {
  std::vector<DeviceInfo> devices;
  boost::system::error_code ec;
  if (not fs::exists(kInputDirectory, ec) or ec or not fs::is_directory(kInputDirectory, ec) or ec) return devices;

  for (fs::directory_iterator it(kInputDirectory, ec), end; not ec and it != end; it.increment(ec)) {
    if (ec) break;

    long js_id                 = -1;
    const std::string filename = it->path().filename().string();
    if (not parseJoystickId(filename, js_id)) continue;
    devices.push_back({js_id, it->path().string()});
  }

  std::sort(devices.begin(), devices.end(),
            [](const DeviceInfo &lhs, const DeviceInfo &rhs) { return lhs.id < rhs.id; });
  return devices;
}

bool UsbJoystick::loadKeymapByName(const std::string &device_name, Keymap &keymap) const {
  std::string name = toLowercase(device_name);
  if (name.find("x-box") != std::string::npos or name.find("xbox") != std::string::npos) {
    keymap = getXboxKeymap();
    return true;
  }

  // Search keymaps from higher-priority config directories to lower-priority ones.
  for (const auto &config_dir : getConfigSearchPaths()) {
    fs::path joystick_dir = fs::path(config_dir) / "joystick";
    if (not fs::exists(joystick_dir) or not fs::is_directory(joystick_dir)) continue;

    for (const auto &entry : fs::directory_iterator(joystick_dir)) {
      if (entry.path().extension() != ".yml") continue;
      std::string stem = toLowercase(entry.path().stem().string());
      if (name.find(stem) != std::string::npos) {
        keymap = Keymap{yml::loadFile(entry.path().string())};
        return true;
      }
    }
  }

  return false;
}

void UsbJoystick::disconnect(bool log_disconnect) {
  bool was_connected = connected_.exchange(false);
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }

  std::string name = device_name_;
  device_path_.clear();
  device_name_.clear();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    slots_ = {};
  }

  if (was_connected and log_disconnect) STEPIT_WARN("Joystick '{}' disconnected.", name);
}

void UsbJoystick::processEvent(const js_event &event) {
  bool is_init = event.type & JS_EVENT_INIT;
  uint8_t type = event.type & ~JS_EVENT_INIT;

  if (type == JS_EVENT_AXIS) {
    updateAxis(event.number, normalizeAxisValue(event.value), not is_init);
  } else if (type == JS_EVENT_BUTTON) {
    updateButton(event.number, event.value != 0, not is_init);
  }
}

void UsbJoystick::updateAxis(std::size_t aid, float value, bool emit_transient) {
  if (aid >= Slots::NAxes) return;

  std::lock_guard<std::mutex> lock(mutex_);
  slots_.axes[aid] = value;

  float value01 = (value + 1.0f) * 0.5f;
  setButtonState(slots_.buttons[Slots::NButtons + 2 * aid], value01 < 0.01f, emit_transient);
  setButtonState(slots_.buttons[Slots::NButtons + 2 * aid + 1], value01 > 0.99f, emit_transient);
}

void UsbJoystick::updateButton(std::size_t bid, bool pressed, bool emit_transient) {
  if (bid >= Slots::NButtons) return;

  std::lock_guard<std::mutex> lock(mutex_);
  setButtonState(slots_.buttons[bid], pressed, emit_transient);
}

STEPIT_REGISTER_JOYSTICK(usb, kDefPriority, Joystick::make<UsbJoystick>);
STEPIT_REGISTER_CTRLINPUT(joystick_usb, kDefPriority, []() { return std::make_unique<JoystickControl>("usb"); });
}  // namespace joystick
}  // namespace stepit

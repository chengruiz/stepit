#ifndef STEPIT_PUBLISHER_H_
#define STEPIT_PUBLISHER_H_

#include <cstddef>
#include <map>
#include <memory>
#include <string>

#include <stepit/data_type.h>
#include <stepit/registry.h>
#include <stepit/robot.h>

namespace stepit {
/**
 * Abstract publishing interface for status, low-level state, and array outputs.
 *
 * Implementations expose process-wide publishing backends through the global
 * registry and singleton accessors.
 */
class Publisher : public Interface<Publisher> {
 public:
  /** Returns the process-wide publisher instance. */
  static Publisher &instance();

  /**
   * Checks whether a named status entry exists.
   *
   * @param name Status key to look up.
   * @return True if the named status exists; otherwise, false.
   */
  bool hasStatus(const std::string &name) const;

  /**
   * Registers a named status entry.
   *
   * @param name Status key to register.
   */
  void registerStatus(const std::string &name);

  /**
   * Updates the current value of a named status entry.
   *
   * @param name Status key to update.
   * @param value New string value for the status entry.
   */
  void updateStatus(const std::string &name, const std::string &value);

  /**
   * Removes a named status entry.
   *
   * @param name Status key to remove.
   */
  void removeStatus(const std::string &name);

  /**
   * Returns a snapshot of all registered status entries.
   *
   * @return Copy of the current status map.
   */
  std::map<std::string, std::string> getStatusSnapshot() const;

  /** Publishes the current status snapshot. */
  virtual void publishStatus() {}

  /**
   * Publishes the latest low-level robot state and command.
   *
   * @param spec Robot specification associated with the data.
   * @param state Latest low-level robot state.
   * @param cmd Latest low-level command buffer.
   */
  virtual void publishLowLevel(const RobotSpec &spec, const LowState &state, const LowCmd &cmd) {}

  /**
   * Publishes a named runtime-typed array.
   *
   * The pointed-to data is borrowed only for the duration of this call.
   * Implementations must copy it before returning.
   *
   * @param name Array channel name.
   * @param data Address of the first scalar, or null when size is zero.
   * @param size Number of scalar elements.
   * @param dtype Scalar data type.
   */
  virtual void publishArray(const std::string &name, const void *data, std::size_t size, DataType dtype) {}

 protected:
  mutable std::mutex status_mutex_;
  std::map<std::string, std::string> named_status_;
};

namespace publisher {
/** Runtime filter that enables or disables individual publisher pathways. */
struct Filter {
  /** Loads filter configuration from the environment. */
  Filter();
  /** Whether status publishing is enabled. */
  bool publish_status{true};
  /** Whether low-level publishing is enabled. */
  bool publish_low_level{true};
  /** Whether array publishing is enabled. */
  bool publish_array{true};
};
/** Process-wide publishing filter. */
extern const Filter g_filter;

/** Returns the process-wide publisher instance. */
inline Publisher &publisher() { return Publisher::instance(); }

/** Returns whether a named status entry exists. */
inline bool hasStatus(const std::string &name) { return publisher().hasStatus(name); }

/** Updates a named status entry with a string value. */
inline void updateStatus(const std::string &name, const std::string &value) { publisher().updateStatus(name, value); }

/** Updates a named status entry with a C-string value. */
inline void updateStatus(const std::string &name, const char *value) { publisher().updateStatus(name, value); }

/** Updates a named status entry using `std::to_string(value)`. */
template <typename T>
void updateStatus(const std::string &name, const T &value) {
  updateStatus(name, std::to_string(value));
}

/** Removes a named status entry. */
inline void removeStatus(const std::string &name) { publisher().removeStatus(name); }

/** Publishes the current status snapshot if status publishing is enabled. */
inline void publishStatus() {
  if (g_filter.publish_status) publisher().publishStatus();
}

/** Publishes low-level data if low-level publishing is enabled. */
inline void publishLowLevel(const RobotSpec &spec, const LowState &low_state, const LowCmd &low_cmd) {
  if (g_filter.publish_low_level) publisher().publishLowLevel(spec, low_state, low_cmd);
}

/** Publishes a runtime-typed array if array publishing is enabled. */
inline void publishArray(const std::string &name, const void *data, std::size_t size, DataType dtype) {
  if (g_filter.publish_array) {
    STEPIT_ASSERT(data != nullptr or size == 0, "Cannot publish non-empty array '{}' from a null pointer.", name);
    STEPIT_ASSERT(dataTypeSize(dtype) > 0, "Cannot publish array '{}' with invalid or undefined data type.", name);
    publisher().publishArray(name, data, size, dtype);
  }
}

/**
 * RAII helper that registers and unregisters one named status entry.
 */
class StatusRegistration {
 public:
  /**
   * Registers a named status entry.
   *
   * @param name Status key to manage.
   */
  StatusRegistration(const std::string &name);
  ~StatusRegistration();
  StatusRegistration(const StatusRegistration &)            = delete;
  StatusRegistration &operator=(const StatusRegistration &) = delete;
  StatusRegistration(StatusRegistration &&other) noexcept;
  StatusRegistration &operator=(StatusRegistration &&other) noexcept;

  /**
   * Updates the managed status entry.
   *
   * @param value Value converted through `updateStatus`.
   */
  template <typename T>
  void update(const T &value) {
    updateStatus(name_, value);
  }

  using Ptr = std::unique_ptr<StatusRegistration>;
  static Ptr make(std::string name) { return std::make_unique<StatusRegistration>(std::move(name)); }

 private:
  std::string name_;
};
}  // namespace publisher
}  // namespace stepit

#define STEPIT_REGISTER_PUBLISHER(name, priority, factory) \
  static ::stepit::Publisher::Registration _publisher_##name##_registration(#name, priority, factory)

#endif  // STEPIT_PUBLISHER_H_

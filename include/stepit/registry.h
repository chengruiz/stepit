#ifndef STEPIT_REGISTRY_H_
#define STEPIT_REGISTRY_H_

#include <algorithm>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>

#include <stepit/logging.h>
#include <stepit/utils.h>

namespace stepit {
constexpr int kMaxPriority = 10;
constexpr int kDefPriority = 5;
constexpr int kMinPriority = 0;

/**
 * @class Registry
 * @brief A thread-safe registry for managing and creating objects via factory functions.
 *
 * The Registry class provides a mechanism to register factory functions associated with
 * specific names and priorities. It allows for the dynamic creation of objects based on
 * these registered names.
 *
 * @tparam T The base type of the objects created by this registry.
 * @tparam Args The argument types required by the factory functions to create objects of type T.
 *
 * @note This class is non-copyable to ensure unique management of the registry state.
 * @note All public operations are thread-safe, protected by an internal mutex.
 */
template <typename T, typename... Args>
class Registry {
 public:
  Registry()                            = default;
  Registry(const Registry &)            = delete;
  Registry &operator=(const Registry &) = delete;

  /**
   * @class Registration
   * @brief Represents a registration entry within the registry system.
   *
   * The Registration class manages the lifecycle of a registered factory function.
   * It registers the factory upon construction and unregisters it upon destruction,
   * providing a RAII-style mechanism to ensure proper cleanup of registry entries.
   */
  class Registration;
  using Factory = std::function<std::unique_ptr<T>(Args...)>;

  std::unique_ptr<T> make(std::string name, Args... args);
  std::string entryListString() const;
  Registration createRegistration(const std::string &name, int priority, Factory factory) {
    return {this, name, priority, std::move(factory)};
  }

 private:
  void insertEntry(std::string name, int priority, Factory factory);
  void eraseEntry(std::string name, int priority);

  struct Entry;
  std::list<Entry> entries_;  // sorted by item priority
  mutable std::recursive_mutex mutex_;
};

template <typename T, typename... Args>
class Registry<T, Args...>::Registration {
 public:
  Registration(Registry *registry, std::string name, int priority, Factory factory);
  ~Registration();
  Registration(const Registration &)            = delete;
  Registration &operator=(const Registration &) = delete;
  Registration(Registration &&other) noexcept;
  Registration &operator=(Registration &&other) noexcept;

 private:
  Registry *registry_;
  std::string name_;
  int priority_;
};

template <typename T, typename... Args>
struct Registry<T, Args...>::Entry {
  Entry() = default;
  Entry(std::string name, int priority, Factory factory)
      : name(std::move(name)), priority(priority), factory(std::move(factory)) {}

  std::string name;
  int priority{kMinPriority};
  Factory factory;
};

template <typename T, typename... Args>
std::unique_ptr<T> Registry<T, Args...>::make(std::string name, Args... args) {
  std::lock_guard<std::recursive_mutex> _(mutex_);
  std::string full_type_name = llu::getTypeName<T>();
  STEPIT_ASSERT(not entries_.empty(), "No factories registered for type '{}'.", full_type_name);

  if (name.empty()) {
    // Get from environment variable
    std::string type_name = full_type_name.substr(full_type_name.find_last_of(':') + 1);
    std::string env_name  = "STEPIT_DEFAULT_" + toUppercase(type_name);
    if (not getenv("_" + env_name, name, false)) getenv(env_name, name);

    if (name.empty()) {
      // Find the highest priority entry
      int highest_priority = entries_.front().priority;
      if (entries_.size() > 1 and highest_priority == (++entries_.begin())->priority) {
        std::string lowercase_type_name = toLowercase(type_name);
        STEPIT_THROW(
            "\n  Multiple factories for type '{}' share the highest priority. Registered factories are: {}\n"
            "  Please disambiguate the factory name by one of the following ways:\n"
            "  1. Setting the {} environment variable to the desired factory name.\n"
            "  2. Passing the factory name to stepit with -f {}@<factory_name>, e.g., -f {}@{}.\n",
            full_type_name, entryListString(), env_name, lowercase_type_name, lowercase_type_name,
            entries_.front().name);
      }
      auto instance = entries_.front().factory(std::forward<Args>(args)...);
      STEPIT_DBUGNT("Created an instance of '{}' for class '{}' with factory named '{}' with the highest priority {}.",
                    getTypeName(*instance), full_type_name, entries_.front().name, entries_.front().priority);
      return instance;
    }
  }

  // Find entries with matching name
  toLowercaseInplace(name);
  auto it = std::find_if(entries_.begin(), entries_.end(), [&name](const Entry &e) { return e.name == name; });
  if (it != entries_.end()) {
    auto instance = it->factory(std::forward<Args>(args)...);
    STEPIT_DBUGNT("Created an instance of '{}' for class '{}' using factory named '{}' with priority {}.",
                  getTypeName(*instance), full_type_name, it->name, it->priority);
    return instance;
  }
  STEPIT_THROW("\n  Factory '{}' not found for type '{}'. Registered factories are: {}", name, full_type_name,
               entryListString());
}

template <typename T, typename... Args>
void Registry<T, Args...>::insertEntry(std::string name, int priority, Factory factory) {
  STEPIT_ASSERT(not name.empty(), "Factory name must not be empty.");
  STEPIT_ASSERT(priority >= kMinPriority and priority <= kMaxPriority, "Priority {} out of range [{}, {}].", priority,
                kMinPriority, kMaxPriority);
  toLowercaseInplace(name);

  std::lock_guard<std::recursive_mutex> _(mutex_);
  // Find the correct insertion point to maintain descending order by priority
  Entry entry(std::move(name), priority, std::move(factory));
  auto it = std::lower_bound(entries_.begin(), entries_.end(), entry, [](const Entry &entry1, const Entry &entry2) {
    return entry1.priority == entry2.priority ? entry1.name < entry2.name : entry1.priority > entry2.priority;
  });
  if (it != entries_.end() and it->name == entry.name and it->priority == entry.priority) {
    STEPIT_THROW("Factory '{}' for type '{}' with priority {} is already registered.", entry.name,
                 llu::getTypeName<T>(), entry.priority);
  }

  entries_.insert(it, entry);
  STEPIT_DBUGNT("Registered factory '{}' of type '{}' with priority {}.", entry.name, llu::getTypeName<T>(),
                entry.priority);
}

template <typename T, typename... Args>
void Registry<T, Args...>::eraseEntry(std::string name, int priority) {
  toLowercaseInplace(name);
  std::lock_guard<std::recursive_mutex> _(mutex_);
  auto it = std::find_if(entries_.begin(), entries_.end(), [&name, priority](const Entry &entry) {
    return entry.name == name && entry.priority == priority;
  });
  if (it != entries_.end()) entries_.erase(it);
}

template <typename T, typename... Args>
std::string Registry<T, Args...>::entryListString() const {
  std::ostringstream oss;
  int priority = -1;
  std::lock_guard<std::recursive_mutex> _(mutex_);
  for (const auto &entry : entries_) {
    if (priority != entry.priority) {
      oss << "\n  - priority " << entry.priority << ": ";
      oss << "'" << entry.name << "'";
      priority = entry.priority;
    } else {
      oss << ", " << "'" << entry.name << "'";
    }
  }
  return oss.str();
}

template <typename T, typename... Args>
Registry<T, Args...>::Registration::Registration(Registry *registry, std::string name, int priority, Factory factory)
    : registry_(registry), name_(std::move(name)), priority_(priority) {
  registry_->insertEntry(name_, priority_, std::move(factory));
}

template <typename T, typename... Args>
Registry<T, Args...>::Registration::~Registration() {
  if (registry_ == nullptr or name_.empty()) return;
  registry_->eraseEntry(name_, priority_);
}

template <typename T, typename... Args>
Registry<T, Args...>::Registration::Registration(Registration &&other) noexcept
    : registry_(other.registry_), name_(std::move(other.name_)), priority_(other.priority_) {
  other.name_.clear();  // Prevent the moved-from object from removing the registry entry
  other.registry_ = nullptr;
}

template <typename T, typename... Args>
auto Registry<T, Args...>::Registration::operator=(Registration &&other) noexcept -> Registration & {
  if (this == &other) return *this;
  name_     = std::move(other.name_);
  priority_ = other.priority_;
  registry_ = other.registry_;
  other.name_.clear();
  other.registry_ = nullptr;
  return *this;
}

/**
 * @class Interface
 * @brief CRTP-style helper that equips a derived interface type with a global registry.
 *
 * This class provides:
 * - A singleton registry for mapping string names to factories.
 * - A convenient RAII registration helper to register implementations.
 * - Factory functions to construct registered implementations by name.
 *
 * @tparam Derived  The concrete interface type inheriting from this helper (CRTP).
 * @tparam Args     Constructor arguments forwarded to registered factories.
 *
 * @par Type aliases
 * - Ptr:      Owning pointer type returned by factory methods (std::unique_ptr<Derived>).
 * - Registry: The underlying ::stepit::Registry used to store factories.
 * - Factory:  The factory callable type defined by Registry.
 *
 * @par Nested types
 * - Registration: RAII object that registers a factory under a name with a priority.
 *
 * @par Factory methods
 * - make(name, args...): Constructs an instance registered under @p name.
 * - make<T>(args...): Constructs a concrete type @p T directly.
 *
 * @warning Registered factories are expected to create objects compatible with Derived.
 * @throws Whatever exceptions may be thrown by the underlying registry/factory.
 */
template <typename Derived, typename... Args>
class Interface {
 public:
  virtual ~Interface() = default;
  using Ptr            = std::unique_ptr<Derived>;
  using Registry       = ::stepit::Registry<Derived, Args...>;
  using Factory        = typename Registry::Factory;

  static Registry &registry() {
    static Registry instance;
    return instance;
  }

  class Registration : public Registry::Registration {
   public:
    Registration(const std::string &name, int priority, Factory factory)
        : Registry::Registration(&registry(), name, priority, std::move(factory)) {}
  };

  static Ptr make(std::string name, Args... args) {
    return registry().make(std::move(name), std::forward<Args>(args)...);
  }

  template <typename T>
  static Ptr make(Args... args) {
    return std::make_unique<T>(std::forward<Args>(args)...);
  }
};
}  // namespace stepit

#endif  // STEPIT_REGISTRY_H_

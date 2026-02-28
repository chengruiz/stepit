#include <dlfcn.h>

#include <stepit/logging.h>
#include <stepit/plugin.h>
#include <stepit/utils.h>

namespace stepit {
PluginManager::PluginManager(std::vector<std::string> &args) {
  STEPIT_ASSERT(args.size() > 0, "At least one argument is required.");
  getenv("STEPIT_BLACKLIST_PLUGINS", blacklist_plugins_);

  // Make C-style argc and argv from args
  int argc    = static_cast<int>(args.size());
  char **argv = new char *[argc + 1];
  for (int i{}; i < argc; ++i) {
    argv[i] = new char[args[i].length() + 1];
    std::strcpy(argv[i], args[i].c_str());
  }
  argv[argc] = nullptr;

  for (const auto &dir : getPluginDirs(args[0])) {
    if (not dir.empty()) loadPlugins(dir, argc, argv);
  }

  // Copy back to args
  args.clear();
  for (int i{}; i < argc; ++i) {
    args.emplace_back(argv[i]);
    delete[] argv[i];
  }
  delete[] argv;
}

PluginManager::~PluginManager() {
  for (void *handle : handles_) {
    auto *cleanup_fn = reinterpret_cast<void (*)()>(dlsym(handle, "stepit_plugin_cleanup"));
    if (cleanup_fn != nullptr) cleanup_fn();
  }
}

void PluginManager::loadPlugins(const std::string &plugin_dir, int &argc, char *argv[]) {
  if (not fs::exists(plugin_dir)) return;
  fs::path absolute_dir = fs::canonical(plugin_dir);
  if (not fs::is_directory(absolute_dir)) {
    STEPIT_WARNNT("Path '{}' is not a directory.", absolute_dir.string());
    return;
  }

  STEPIT_DBUGNT("Loading plugins from directory '{}'.", absolute_dir.string());
  for (const auto &entry : fs::directory_iterator(absolute_dir)) {
    if (not fs::is_regular_file(entry)) continue;

    std::string filename = entry.path().filename().string();
    if (isValidPlugin(filename)) {
      if (isBlacklistedPlugin(filename)) {
        STEPIT_DBUGNT("-- Skipping blacklisted plugin '{}'.", filename);
        continue;
      }

      STEPIT_DBUGNT("-- Loading plugin '{}'.", filename);
      std::string full_path = entry.path().string();

      void *handle = dlopen(full_path.c_str(), RTLD_NOW | RTLD_GLOBAL);
      if (handle == nullptr) {
        STEPIT_WARNNT("During loading plugin '{}':\n  {}.", filename, dlerror());
        continue;
      }

      auto *init_fn = reinterpret_cast<int (*)(int &, char *[])>(dlsym(handle, "stepit_plugin_init"));
      if (init_fn != nullptr) {
        int ret = init_fn(argc, argv);
        if (ret != 0) {
          STEPIT_WARNNT("Failed to initialize '{}' (error code: {}).", filename, ret);
          dlclose(handle);
          continue;
        }
      }

      handles_.push_back(handle);
    }
  }
}

std::vector<std::string> PluginManager::retrievePluginArgs(int &argc, char *argv[]) {
  int original_argc = argc;
  std::vector<std::string> plugin_args{argv[0]};
  for (int i{1}; i < original_argc; ++i) {
    if (std::strcmp(argv[i], "--") == 0) {
      argc = i;
      for (int j{i + 1}; j < original_argc; ++j) {
        plugin_args.emplace_back(argv[j]);
      }
      break;
    }
  }
  return plugin_args;
}

std::vector<std::string> PluginManager::getPluginDirs(const std::string &executable_path) {
  std::vector<std::string> plugin_dirs;
  if (not getenv("STEPIT_PLUGIN_DIRS", plugin_dirs)) {
    getenv("STEPIT_EXTRA_PLUGIN_DIRS", plugin_dirs);
    fs::path executable_dir = fs::path(executable_path).parent_path();
    plugin_dirs.push_back(executable_dir.string());                  // ROS2 pattern
    plugin_dirs.push_back((executable_dir / "../lib").string());     // CMake pattern
    plugin_dirs.push_back((executable_dir / "../../lib").string());  // ROS1 pattern
  }
  return plugin_dirs;
}

bool PluginManager::isValidPlugin(const std::string &filename) {
  std::size_t prefix_len = std::strlen(kPluginPrefix);
  std::size_t suffix_len = std::strlen(kPluginSuffix);
  std::size_t total_len  = prefix_len + suffix_len;

  if (filename.length() <= total_len) return false;
  if (filename.compare(0, prefix_len, kPluginPrefix) != 0) return false;
  if (filename.compare(filename.length() - suffix_len, suffix_len, kPluginSuffix) != 0) return false;
  return true;
}

std::string PluginManager::getPluginName(const std::string &path) {
  std::string name = fs::path(path).filename().string();
  if (name.empty()) return name;

  const std::size_t prefix_len = std::strlen(kPluginPrefix);
  const std::size_t suffix_len = std::strlen(kPluginSuffix);

  if (name.size() > prefix_len && name.compare(0, prefix_len, kPluginPrefix) == 0) {
    name = name.substr(prefix_len);
  }
  if (name.size() > suffix_len && name.compare(name.size() - suffix_len, suffix_len, kPluginSuffix) == 0) {
    name = name.substr(0, name.size() - suffix_len);
  }
  return name;
}

bool PluginManager::isBlacklistedPlugin(const std::string &filename) {
  if (blacklist_plugins_.empty()) return false;

  std::string plugin_name = getPluginName(filename);
  for (const auto &blacklisted : blacklist_plugins_) {
    if (blacklisted == filename) return true;
    if (getPluginName(blacklisted) == plugin_name) return true;
  }
  return false;
}
}  // namespace stepit

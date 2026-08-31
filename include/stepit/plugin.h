#ifndef STEPIT_PLUGIN_H_
#define STEPIT_PLUGIN_H_

#include <string>
#include <vector>

namespace stepit {
constexpr const char *kPluginPrefix      = "libstepit_plugin_";
constexpr const char *kPluginSuffix      = ".so";
constexpr const char *kPluginEntrySuffix = "_entry";

class PluginManager {
 public:
  explicit PluginManager(std::vector<std::string> &args);
  ~PluginManager();
  void loadPlugins(const std::string &plugin_dir, int &argc, char *argv[]);
  static std::vector<std::string> retrievePluginArgs(int &argc, char *argv[]);

 private:
  static std::vector<std::string> getPluginDirs(const std::string &executable_path);
  /**
   * Checks whether the given filename matches the expected plugin naming convention.
   *
   * A valid plugin filename must start with "libstepit_plugin_", end with ".so",
   * and contain a non-empty name between the prefix and suffix.
   *
   * @param filename File name to validate.
   * @return True if the filename is a valid plugin name; otherwise, false.
   */
  static bool isValidPlugin(const std::string &filename);
  static std::string getPluginName(const std::string &plugin);
  bool isBlacklistedPlugin(const std::string &filename);

  std::vector<void *> handles_;
  std::vector<std::string> blacklist_plugins_;
};
}  // namespace stepit

#endif  // STEPIT_PLUGIN_H_

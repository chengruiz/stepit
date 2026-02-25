#include <stepit/utils.h>

namespace stepit {
std::string getGlobalConfigDir(const std::string &relative_path) {
  static std::string config_dir;
  if (config_dir.empty()) {
    if (not getenv("STEPIT_CONFIG_DIR", config_dir)) {
#ifdef STEPIT_CONFIG_DIR
      config_dir = STEPIT_CONFIG_DIR;
#else
      const char *home_dir = std::getenv("HOME");

      config_dir = joinPaths(home_dir != nullptr ? home_dir : ".", ".config", "stepit");
#endif  // STEPIT_CONFIG_DIR
    }
  }
  return relative_path.empty() ? config_dir : joinPaths(config_dir, relative_path);
}

YAML::Node loadGlobalConfigYaml(const std::string &relative_path) {
  return YAML::LoadFile(getGlobalConfigDir(relative_path));
}
}  // namespace stepit

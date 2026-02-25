#ifndef STEPIT_UTILS_H_
#define STEPIT_UTILS_H_

#include <algorithm>
#include <numeric>

#include <boost/filesystem.hpp>

#include <llu/chrono.h>
#include <llu/env.h>
#include <llu/error.h>
#include <llu/geometry.h>
#include <llu/math.h>
#include <llu/range.h>
#include <llu/ring.h>
#include <llu/typename.h>
#include <llu/yaml.h>
#include <stepit/logging.h>

namespace stepit {
using namespace llu;
namespace fs = boost::filesystem;

template <typename T>
bool getenv(const char *name, T &result, bool verbose = true) {
  if (llu::getenv(name, result)) {
    if (verbose) STEPIT_INFONT("Env: Read {} = {}.", name, result);
    return true;
  }
  return false;
}

template <typename T>
bool getenv(const std::string &name, T &result, bool verbose = true) {
  return getenv(name.c_str(), result, verbose);
}

std::string getGlobalConfigDir(const std::string &relative_path = "");

YAML::Node loadGlobalConfigYaml(const std::string &relative_path);

template <typename Key, typename Value>
Value lookupMap(const Key &key, const std::map<Key, Value> &map) {
  auto it = map.find(key);
  STEPIT_ASSERT(it != map.end(), "Key '{}' not found.", key);
  return it->second;
}

template <typename Key, typename Value>
Value lookupMap(const Key &key, const std::map<Key, Value> &map, const Value &default_value) {
  auto it = map.find(key);
  return it == map.end() ? default_value : it->second;
}

template <typename T, std::size_t N>
std::vector<T> array2vector(const std::array<T, N> &arr) {
  return std::vector<T>(arr.begin(), arr.end());
}

inline std::string nonEmptyOr(const std::string &str1, const std::string &str2) { return str1.empty() ? str2 : str1; }

inline void appendPaths(fs::path &) {}

template <typename Path, typename... Paths>
inline void appendPaths(fs::path &result, const Path &path, const Paths &...paths) {
  result /= fs::path(path);
  appendPaths(result, paths...);
}

template <typename... Paths>
std::string joinPaths(const std::string &path1, const std::string &path2, const Paths &...paths) {
  fs::path result(path1);
  result /= fs::path(path2);
  appendPaths(result, paths...);
  return result.string();
}

inline std::string addExtensionIfMissing(const std::string &path, const std::string &extension) {
  if (fs::path(path).extension().empty()) return path + extension;
  return path;
}

inline std::string replaceExtension(const std::string &path, const std::string &extension) {
  return fs::path(path).replace_extension(extension).string();
}

template <typename T>
T product(const std::vector<T> &vec) {
  return std::accumulate(vec.begin(), vec.end(), static_cast<T>(1), std::multiplies<>());
}
}  // namespace stepit

#ifdef STEPIT_FIX_GETTID
#include <sys/syscall.h>
#define gettid() syscall(SYS_gettid)
#endif  // STEPIT_FIX_GETTID

#endif  // STEPIT_UTILS_H_

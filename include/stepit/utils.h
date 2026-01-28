#ifndef STEPIT_UTILS_H_
#define STEPIT_UTILS_H_

#include <boost/filesystem.hpp>

#include <llu/chrono.h>
#include <llu/env.h>
#include <llu/error.h>
#include <llu/geometry.h>
#include <llu/math.h>
#include <llu/range.h>
#include <llu/squeue.h>
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

std::string getConfigPath(const std::string &relative_path = "");

YAML::Node loadConfigFile(const std::string &relative_path);
}  // namespace stepit

#ifdef STEPIT_FIX_GETTID
#include <sys/syscall.h>
#define gettid() syscall(SYS_gettid)
#endif  // STEPIT_FIX_GETTID

#endif  // STEPIT_UTILS_H_

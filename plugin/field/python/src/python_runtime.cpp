#include <memory>
#include <mutex>

#include <pybind11/embed.h>

#include <stepit/pyutils/python_runtime.h>

namespace stepit {
namespace {
auto &interpreterMutex() {
  static std::mutex mutex;
  return mutex;
}

auto &interpreterHolder() {
  static std::unique_ptr<pybind11::scoped_interpreter> holder;
  return holder;
}
}  // namespace

void ensurePythonInterpreter() {
  std::lock_guard<std::mutex> lock(interpreterMutex());
  auto &holder = interpreterHolder();
  if (holder == nullptr and not Py_IsInitialized()) {
    holder = std::make_unique<pybind11::scoped_interpreter>();
  }
}

void releasePythonInterpreter() {
  std::lock_guard<std::mutex> lock(interpreterMutex());
  interpreterHolder().reset();
}
}  // namespace stepit

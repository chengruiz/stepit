#include <memory>
#include <pybind11/embed.h>

static std::unique_ptr<pybind11::scoped_interpreter> g_interpreter;

extern "C" {
int stepit_plugin_init(int &argc, char **argv) {
  if (not Py_IsInitialized()) {
    g_interpreter = std::make_unique<pybind11::scoped_interpreter>();
  }
  return 0;
}

int stepit_plugin_cleanup(int &argc, char **argv) {
  g_interpreter.reset();
  return 0;
}
}

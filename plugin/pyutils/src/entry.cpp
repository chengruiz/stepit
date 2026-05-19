#include <stepit/pyutils/python_runtime.h>

extern "C" {
int stepit_plugin_init(int &argc, char **argv) {
  stepit::ensurePythonInterpreter();
  return 0;
}

void stepit_plugin_cleanup() {
  stepit::releasePythonInterpreter();
}
}

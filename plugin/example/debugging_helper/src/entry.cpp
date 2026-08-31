#include <stepit/logging.h>

extern "C" {
int stepit_plugin_init(int &argc, char **argv) {  // For testing purpose.
  for (int i{}; i < argc; ++i) {
    if (std::strcmp(argv[i], "__debugging") == 0) {
      STEPIT_THROW("The plugin `debugging_helper` is successfully initialized.");
    }
  }
  return 0;
}
}

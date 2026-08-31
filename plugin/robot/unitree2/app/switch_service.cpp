#include <iostream>
#include <string>

#include <stepit/robot/unitree2/common.h>

void printUsage(const char *prog) {
  std::cout << "Usage: " << prog << " <subcommand> [args]\n";
  std::cout << "Subcommands:\n";
  std::cout << "  help                      Produce help message\n";
  std::cout << "  switch <service> <state>  Switch service state, state is on/off\n";
  std::cout << "  enable <service>          Enable service\n";
  std::cout << "  disable <service>         Disable service\n";
}

bool parseState(const std::string &state, bool &enable) {
  if (state == "on" or state == "enable" or state == "enabled" or state == "1" or state == "true") {
    enable = true;
    return true;
  }
  if (state == "off" or state == "disable" or state == "disabled" or state == "0" or state == "false") {
    enable = false;
    return true;
  }
  return false;
}

#define EXIT_IF(condition) \
  do {                     \
    if (condition) {       \
      printUsage(argv[0]); \
      return 1;            \
    }                      \
  } while (0)

int main(int argc, char **argv) {
  using stepit::Unitree2ServiceSwitcher;
  EXIT_IF(argc < 2);

  std::string cmd = argv[1];

  if (cmd == "help" or cmd == "-h" or cmd == "--help") {
    EXIT_IF(argc != 2);
    printUsage(argv[0]);
    return 0;
  }

  Unitree2ServiceSwitcher::initialize();

  if (cmd == "switch") {
    EXIT_IF(argc != 4);

    bool enable = false;
    if (not parseState(argv[3], enable)) {
      std::cerr << "Unknown service state: " << argv[3] << std::endl;
      printUsage(argv[0]);
      return 1;
    }

    Unitree2ServiceSwitcher::serviceSwitch(argv[2], enable);
    return 0;
  }

  if (cmd == "enable") {
    EXIT_IF(argc != 3);
    Unitree2ServiceSwitcher::serviceSwitch(argv[2], true);
    return 0;
  }

  if (cmd == "disable") {
    EXIT_IF(argc != 3);
    Unitree2ServiceSwitcher::serviceSwitch(argv[2], false);
    return 0;
  }

  std::cerr << "Unknown subcommand: " << cmd << std::endl;
  printUsage(argv[0]);
  return 1;
}

#include <iostream>
#include <string>

#include <stepit/robot/unitree2/common.h>

void printUsage(const char *prog) {
  std::cout << "Usage: " << prog << " <subcommand> [args]\n";
  std::cout << "Subcommands:\n";
  std::cout << "  help              Produce help message\n";
  std::cout << "  activate <mode>   Activate locomotion\n";
  std::cout << "  deactivate        Deactivate locomotion\n";
  std::cout << "  disable           Disable locomotion permanently\n";
  std::cout << "  enable            Enable locomotion permanently\n";
  std::cout << "  status            Show current status\n";
}

#define EXIT_IF(condition) \
  do {                     \
    if (condition) {       \
      printUsage(argv[0]); \
      return 1;            \
    }                      \
  } while (0)

int main(int argc, char **argv) {
  using stepit::Unitree2MotionSwitcher;
  EXIT_IF(argc < 2);

  std::string cmd = argv[1];

  if (cmd == "help" or cmd == "-h" or cmd == "--help") {
    EXIT_IF(argc != 2);
    printUsage(argv[0]);
    return 0;
  }

  Unitree2MotionSwitcher::initialize();

  if (cmd == "activate") {
    EXIT_IF(argc != 3);
    Unitree2MotionSwitcher::activate(argv[2]);
    return 0;
  }

  if (cmd == "deactivate") {
    EXIT_IF(argc != 2);
    Unitree2MotionSwitcher::deactivate();
    return 0;
  }

  if (cmd == "disable") {
    EXIT_IF(argc != 2);
    Unitree2MotionSwitcher::disable();
    return 0;
  }

  if (cmd == "enable") {
    EXIT_IF(argc != 2);
    Unitree2MotionSwitcher::enable();
    return 0;
  }

  if (cmd == "status") {
    EXIT_IF(argc != 2);
    Unitree2MotionSwitcher::status();
    return 0;
  }

  std::cerr << "Unknown subcommand: " << cmd << std::endl;
  printUsage(argv[0]);
  return 1;
}

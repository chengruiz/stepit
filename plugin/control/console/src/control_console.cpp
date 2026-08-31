#include <poll.h>
#include <unistd.h>
#include <iostream>
#include <string>

#include <stepit/registry.h>
#include <stepit/control_console/control_console.h>

namespace stepit {
ConsoleControl::ConsoleControl() : status_(true), reader_thread_([this] { readerThread(); }) {}

ConsoleControl::~ConsoleControl() {
  status_ = false;
  if (reader_thread_.joinable()) reader_thread_.join();
}

void ConsoleControl::readerThread() {
  std::string line;
  while (status_) {
    // Check if there is any input
    pollfd pfd{STDIN_FILENO, POLLIN, 0};
    int ret = ::poll(&pfd, 1, 50 /* ms */);
    if (ret == 0) continue;
    if (ret < 0 or not std::getline(std::cin, line)) {
      status_ = false;
      break;
    }

    // Trim whitespace from both ends
    auto start = line.find_first_not_of(" \t\n\r");
    if (start == std::string::npos) continue;
    auto end = line.find_last_not_of(" \t\n\r");
    put(line.substr(start, end - start + 1));
  }
}

STEPIT_REGISTER_CTRLINPUT(console, kDefPriority, ControlInput::make<ConsoleControl>);
}  // namespace stepit

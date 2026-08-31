#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <exception>
#include <iostream>
#include <thread>

#include <boost/program_options.hpp>
#include <fmt/format.h>

#include <stepit/plugin.h>
#include <stepit/spin.h>
#include <stepit/joystick/joystick.h>

using namespace stepit;
namespace po = boost::program_options;

namespace {
class JoystickStatePrinter {
 public:
  JoystickStatePrinter(std::string factory) : factory_(std::move(factory)) {}
  void start() { worker_ = std::thread(&JoystickStatePrinter::run, this); }
  void stop() { running_ = false; }
  void join() {
    if (worker_.joinable()) worker_.join();
    if (worker_exception_) std::rethrow_exception(worker_exception_);
  }

 private:
  void run() {
    try {
      auto js = joystick::Joystick::make(factory_);
      joystick::State state;
      bool last_connected = false;
      std::string last_line;

      std::cout << "Polling joystick";
      if (not factory_.empty()) std::cout << " with factory '" << factory_ << "'";
      std::cout << ". Press Ctrl+C to exit." << std::endl;

      while (running_.load()) {
        const bool connected = js->connected();
        if (connected) {
          js->getState(state);
          const std::string line = fmt::format("{}", state);
          if (not last_connected || line != last_line) {
            std::cout << line << std::endl;
            last_line = line;
          }
        } else if (last_connected) {
          std::cout << "Joystick disconnected. Waiting for reconnect..." << std::endl;
          last_line.clear();
        } else if (last_line.empty()) {
          std::cout << "Waiting for joystick connection..." << std::endl;
          last_line = "waiting";
        }

        last_connected = connected;
        std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms_));
      }
    } catch (...) {
      worker_exception_ = std::current_exception();
      std::raise(SIGINT);
    }
  }

  std::string factory_;
  int interval_ms_{10};
  std::atomic<bool> running_{true};
  std::thread worker_;
  std::exception_ptr worker_exception_;
};
}  // namespace

int main(int argc, char *argv[]) {
  po::options_description arg_desc("Allowed arguments");
  // clang-format off
  arg_desc.add_options()
      ("help,h",
          "Show this help message")
      ("factory,f", po::value<std::string>()->default_value(""),
          "Joystick factory name")
      ("verbosity,v", po::value<int>(),
          "Verbosity level (0-3)")
      (" arg1 arg2 ...",
          "Plugins arguments (after '--')")
      ;
  // clang-format on

  auto plugin_args = PluginManager::retrievePluginArgs(argc, argv);

  po::variables_map arg_map;
  po::parsed_options parsed_args = po::command_line_parser(argc, argv).options(arg_desc).allow_unregistered().run();
  po::store(parsed_args, arg_map);
  if (arg_map.find("help") != arg_map.end()) {
    std::cout << arg_desc << std::endl;
    return 0;
  }
  po::notify(arg_map);

  std::vector<std::string> unrecognized_args = po::collect_unrecognized(parsed_args.options, po::include_positional);
  STEPIT_ASSERT(unrecognized_args.empty(), "Unrecognized arguments: {}.", unrecognized_args);

  if (arg_map.find("verbosity") != arg_map.end()) {
    STEPIT_SET_VERBOSITY(static_cast<VerbosityLevel>(arg_map["verbosity"].as<int>()));
  }

  PluginManager plugin_manager(plugin_args);

  std::string factory = arg_map["factory"].as<std::string>();
  if (startsWith(factory, "joystick@")) {
    factory = factory.substr(std::strlen("joystick@"));
  } else if (factory.find('@') != std::string::npos) {
    fmt::print(std::cerr, "{} Invalid factory name '{}'. Expected a factory name of joystick.\n", kErrorPrefix,
               factory);
    return -1;
  }

  try {
    JoystickStatePrinter printer(factory);
    printer.start();
    const int spin_result = stepit::spin();
    printer.stop();
    printer.join();
    if (spin_result != 0) {
      return spin_result;
    }
  } catch (const std::exception &e) {
    fmt::print(std::cerr, "{}: {}\n", kErrorPrefix, e.what());
    return 1;
  }

  std::cout << "Stopped." << std::endl;
  return 0;
}

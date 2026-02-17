#include <iostream>
#include <string>
#include <vector>
#include <boost/program_options.hpp>

#include <stepit/utils.h>

namespace po = boost::program_options;
using namespace llu;

void printUsage() {
  std::cout << "Usage: rotation_transform -i <quaternion | euler | matrix> "
               "-o <quaternion | euler | matrix> [--degree] <values>"
            << std::endl;
}

#define EXIT_IF(condition, message)                                     \
  do {                                                                  \
    if (condition) {                                                    \
      std::cerr << kRed << "ERROR: " << kClear << message << std::endl; \
      return 1;                                                         \
    }                                                                   \
  } while (0)

int main(int argc, char *argv[]) {
  po::options_description arg_desc("Allowed arguments");
  // clang-format off
  arg_desc.add_options()
      ("help,h", "")
      ("input,i", po::value<std::string>()->required())
      ("output,o", po::value<std::string>()->required())
      ("degree", po::bool_switch(), "Use degrees instead of radians for input")
      ("value", po::value<std::vector<double>>()->required())
      ;
  // clang-format on

  po::positional_options_description pos_desc;
  pos_desc.add("value", -1);

  po::variables_map vm;
  try {
    auto parsed = po::command_line_parser(argc, argv).options(arg_desc).positional(pos_desc).run();
    po::store(parsed, vm);
    if (vm.count("help")) {
      printUsage();
      return 0;
    }
    po::notify(vm);
  } catch (const std::exception &e) {
    printUsage();
    return 1;
  }

  auto in_type  = vm["input"].as<std::string>();
  auto out_type = vm["output"].as<std::string>();
  auto value    = vm["value"].as<std::vector<double>>();
  bool degree   = vm["degree"].as<bool>();

  Quatd quaterion;
  if (std::string("quaternion").rfind(in_type) != std::string::npos) {
    EXIT_IF(value.size() != 4, "Quaternion requires 4 values.");
    quaterion = Quatd(value[0], value[1], value[2], value[3]);
  } else if (std::string("euler").rfind(in_type) != std::string::npos) {
    EXIT_IF(value.size() != 3, "Euler angles require 3 values.");
    if (degree) {
      value[0] = deg2rad(value[0]);
      value[1] = deg2rad(value[1]);
      value[2] = deg2rad(value[2]);
    }
    quaterion = Quatd::fromEulerAngles(value[0], value[1], value[2]);
  } else if (std::string("matrix").rfind(in_type) != std::string::npos) {
    EXIT_IF(value.size() != 9, "Matrix requires 9 values.");
    Mat3d m;
    for (int i{}; i < 9; ++i) m(i / 3, i % 3) = value[i];
    quaterion = Quatd::fromMatrix(m);
  } else {
    EXIT_IF(true, "Unknown input type '" << in_type << "'.");
  }

  if (std::string("quaternion").rfind(out_type) != std::string::npos) {
    std::cout << "Quaternion: " << "w = " << quaterion.w() << ", x = " << quaterion.x() << ", y = " << quaterion.y()
              << ", z = " << quaterion.z() << std::endl;
  } else if (std::string("euler").rfind(out_type) != std::string::npos) {
    auto euler = quaterion.eulerAngles();
    std::cout << "Euler angles: "
              << "roll = " << euler[0] << "rad (" << euler[0] * 180 / M_PI << "°), "
              << "pitch = " << euler[1] << "rad (" << euler[1] * 180 / M_PI << "°), "
              << "yaw = " << euler[2] << "rad (" << euler[2] * 180 / M_PI << "°)"
              << " in ZYX order" << std::endl;
  } else if (std::string("matrix").rfind(out_type) != std::string::npos) {
    std::cout << "Rotation matrix:\n" << quaterion.matrix() << std::endl;
  } else {
    EXIT_IF(true, "Unknown output type '" << out_type << "'.");
  }

  return 0;
}

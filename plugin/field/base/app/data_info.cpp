#include <algorithm>
#include <iostream>
#include <vector>

#include <boost/program_options.hpp>

#include <stepit/plugin.h>
#include <stepit/field/data_loader.h>

using namespace stepit;
namespace po = boost::program_options;

int main(int argc, char *argv[]) {
  po::options_description arg_desc("Allowed arguments");
  // clang-format off
  arg_desc.add_options()
      ("help,h",
          "Show this help message")
      ("factory,f", po::value<std::string>()->default_value(""),
          "DataLoader factory name")
      ("data_path", po::value<std::string>(),
          "Path to the source data")
      (" arg1 arg2 ...",
          "Plugins arguments (after '--')")
      ;
  // clang-format on

  po::positional_options_description positional_desc;
  positional_desc.add("data_path", 1);

  auto plugin_args = PluginManager::retrievePluginArgs(argc, argv);

  po::variables_map arg_map;
  po::store(po::command_line_parser(argc, argv).options(arg_desc).positional(positional_desc).run(), arg_map);
  if (arg_map.find("help") != arg_map.end()) {
    std::cout << arg_desc << std::endl;
    return 0;
  }
  po::notify(arg_map);
  if (arg_map.find("data_path") == arg_map.end()) {
    std::cerr << "Missing required argument: <data_path>\n" << arg_desc << std::endl;
    return -1;
  }

  PluginManager plugin_manager(plugin_args);

  try {
    auto factory    = arg_map["factory"].as<std::string>();
    const auto path = arg_map["data_path"].as<std::string>();
    if (startsWith(factory, "dataloader@")) {
      factory = factory.substr(std::strlen("dataloader@"));
    } else if (factory.find("@") != std::string::npos) {
      fmt::print(std::cerr, "{} Invalid factory name '{}'. Expected a factory name of dataloader.\n", kErrorPrefix,
                 factory);
      return -1;
    }
    auto data = field::DataLoader::make(factory, path);

    std::size_t max_name_len  = 4;
    std::size_t max_shape_len = 5;
    const auto &names         = data->keys();
    std::vector<std::string> shape_strs;
    std::vector<std::string> dtype_strs;

    for (const auto &name : names) {
      const auto &array = (*data)[name];
      max_name_len      = std::max(max_name_len, name.length());
      shape_strs.push_back(fmt::format("({})", fmt::join(array.shape, ", ")));
      max_shape_len = std::max(max_shape_len, shape_strs.back().length());
      dtype_strs.push_back(array.dtype);
    }

    max_name_len += 2;
    max_shape_len += 2;
    std::cout << fmt::format("{:<{}}{:<{}}{}\n", "Name", max_name_len, "Shape", max_shape_len, "Dtype")
              << std::string(max_name_len + max_shape_len + 15, '-') << std::endl;

    for (std::size_t i{}; i < names.size(); ++i) {
      std::cout << fmt::format("{:<{}}{:<{}}{}", names[i], max_name_len, shape_strs[i], max_shape_len, dtype_strs[i])
                << std::endl;
    }
  } catch (const std::exception &e) {
    fmt::print(std::cerr, "{}: {}\n", kErrorPrefix, e.what());
    return 1;
  }

  return 0;
}

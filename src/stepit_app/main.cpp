#include <iostream>
#include <boost/program_options.hpp>

#include <stepit/agent.h>
#include <stepit/plugin.h>

using namespace stepit;
namespace po = boost::program_options;

int main(int argc, char *argv[]) {
  po::options_description arg_desc("Allowed arguments");
  // clang-format off
  arg_desc.add_options()
      ("help,h",
          "Show this help message")
      ("control,c", po::value<std::vector<std::string>>()->composing(),
          "Control input type (repeatable)")
      ("factory,f", po::value<std::vector<std::string>>()->composing(),
          "Default factory (repeatable, format: <class>@<factory_name>)")
      ("publisher,P", po::value<std::string>(),
          "Publisher type")
      ("policy,p", po::value<std::vector<std::string>>()->composing(),
          "Policies (format: [<type>@]<directory>)")
      ("robot,r", po::value<std::string>()->default_value(""),
          "Robot type")
      ("verbosity,v", po::value<int>(),
          "Verbosity level (0-3)")
      (" arg1 arg2 ...",
          "Plugins arguments (after '--')")
      ;
  // clang-format on

  // Pass arguments after "--" to plugins
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

  if (arg_map.find("factory") != arg_map.end()) {
    auto factories = arg_map["factory"].as<std::vector<std::string>>();
    for (const auto &factory : factories) {
      auto sep = factory.find('@');
      STEPIT_ASSERT(sep != std::string::npos, "Argument 'factory' should be in the format '<class>@<factory_name>'.");
      std::string type_name    = factory.substr(0, sep);
      std::string factory_name = factory.substr(sep + 1);
      std::string env_name     = "_STEPIT_DEFAULT_" + toUppercase(type_name);
      setenv(env_name.c_str(), factory_name.c_str(), 1);
    }
  }

  if (arg_map.find("publisher") != arg_map.end()) {
    auto publisher_type = arg_map["publisher"].as<std::string>();
    setenv("_STEPIT_DEFAULT_PUBLISHER", publisher_type.c_str(), 1);
  }

  PluginManager plugin_manager(plugin_args);
  std::string robot_type;
  std::vector<std::string> ctrl_type;
  if (arg_map.find("robot") != arg_map.end()) {
    robot_type = arg_map["robot"].as<std::string>();
  }
  if (arg_map.find("control") != arg_map.end()) {
    ctrl_type = arg_map["control"].as<std::vector<std::string>>();
  }
  auto agent = std::make_unique<Agent>(robot_type, ctrl_type);

  // Load and add all specified policies
  if (arg_map.find("policy") != arg_map.end()) {
    auto policies = arg_map["policy"].as<std::vector<std::string>>();
    for (const auto &policy : policies) {
      std::string policy_type, home_dir;
      auto sep = policy.find('@');
      if (sep != std::string::npos) {
        policy_type = policy.substr(0, sep);
        home_dir    = policy.substr(sep + 1);
      } else {
        policy_type = "";
        home_dir    = policy;
      }
      agent->addPolicy(policy_type, home_dir);
    }
  }

  return agent->stepit();
}

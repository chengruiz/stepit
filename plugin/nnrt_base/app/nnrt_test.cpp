#include <iostream>

#include <boost/program_options.hpp>

#include <stepit/plugin.h>
#include <stepit/nnrt/nnrt.h>

using namespace stepit;
namespace po = boost::program_options;

int main(int argc, char *argv[]) {
  po::options_description arg_desc("Allowed arguments");
  // clang-format off
  arg_desc.add_options()
      ("help,h",
          "Show this help message")
      ("factory,f", po::value<std::string>()->default_value(""),
          "NnrtApi factory name")
      ("model_path", po::value<std::string>(),
          "Path to the model")
      ("config_path", po::value<std::string>(),
          "YAML configuration file for the model")
      (" arg1 arg2 ...",
          "Plugins arguments (after '--')")
      ;
  // clang-format on

  po::positional_options_description positional_desc;
  positional_desc.add("model_path", 1);
  positional_desc.add("config_path", 1);

  // Pass arguments after "--" to plugins
  auto plugin_args = PluginManager::retrievePluginArgs(argc, argv);

  po::variables_map arg_map;
  po::store(po::command_line_parser(argc, argv).options(arg_desc).positional(positional_desc).run(), arg_map);
  if (arg_map.find("help") != arg_map.end()) {
    std::cout << arg_desc << std::endl;
    return 0;
  }
  po::notify(arg_map);
  if (arg_map.find("model_path") == arg_map.end()) {
    std::cerr << "Missing required argument: <model_path>\n" << arg_desc << std::endl;
    return -1;
  }

  PluginManager plugin_manager(plugin_args);

  const auto factory = arg_map["factory"].as<std::string>();
  const auto path    = arg_map["model_path"].as<std::string>();
  const auto config  = (arg_map.find("config_path") != arg_map.end())
                           ? yml::loadFile(arg_map["config_path"].as<std::string>())
                           : YAML::Node();

  auto model1 = NnrtApi::make(factory, path, config);
  auto model2 = NnrtApi::make(factory, path, config);
  model1->printInfo();
  model1->clearState();
  model2->clearState();

  displayFormattedBanner(60, nullptr, "Inference test");
  for (std::size_t step{}; step < 3; ++step) {
    fmt::print("Step {} (input {}):\n", step, 0.01F * step);
    // Set inputs
    std::vector<std::vector<float>> inputs;
    inputs.resize(model1->getNumInputs());
    for (std::size_t i{}; i < model1->getNumInputs(); ++i) {
      if (not model1->isInputRecurrent(i)) {
        inputs[i] = std::vector<float>(model1->getInputSize(i), 0.01F * static_cast<float>(step));
        model1->setInput(i, inputs[i].data());
        model2->setInput(i, inputs[i].data());
      }
    }

    // Run inference
    model1->runInference();
    model2->runInference();

    // Check outputs
    for (std::size_t i{}; i < model1->getNumOutputs(); ++i) {
      auto output1 = cmArrXf(model1->getOutput(i), static_cast<Eigen::Index>(model1->getOutputSize(i)));
      auto output2 = cmArrXf(model2->getOutput(i), static_cast<Eigen::Index>(model2->getOutputSize(i)));
      if (not output1.isApprox(output2)) {
        std::cerr << fmt::format("{}ERROR{}: Output '{}' is not consistent.", kRed, kClear, model1->getOutputName(i));
        return -1;
      }
      if (not model1->isOutputRecurrent(i)) {
        std::cout << fmt::format("Output '{}':", model1->getOutputName(i)) << output1.transpose() << std::endl;
      }
    }
  }
  return 0;
}

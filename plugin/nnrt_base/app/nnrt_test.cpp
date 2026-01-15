#include <iostream>

#include <stepit/plugin.h>
#include <stepit/nnrt/nnrt.h>

using namespace stepit;

int main(int argc, char *argv[]) {
  if (argc != 2 and argc != 3) {
    std::cout << "Usage: nn-test <model_path> [model_config_yaml_file]" << std::endl;
    return -1;
  }
  std::vector<std::string> args{argv[0]};
  PluginManager plugin_manager(args);

  auto model1 = NnrtApi::make("", argv[1], argc == 3 ? yml::loadFile(argv[2]) : YAML::Node());
  auto model2 = NnrtApi::make("", argv[1], argc == 3 ? yml::loadFile(argv[2]) : YAML::Node());
  model1->printInfo();
  model1->clearState();
  model2->clearState();

  displayFormattedBanner(60, nullptr, "Inference test");
  for (std::size_t step{}; step < 3; ++step) {
    std::cout << "Step " << step << std::endl;
    for (std::size_t i{}; i < model1->getNumInputs(); ++i) {
      if (not model1->isInputRecurrent(i)) {
        std::vector<float> input_data(model1->getInputSize(i), 0.01F * static_cast<float>(step));
        model1->setInput(i, input_data.data());
        model2->setInput(i, input_data.data());
      }
    }
    model1->runInference();
    model2->runInference();
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

#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>

#include <boost/program_options.hpp>
#include <fmt/core.h>

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
          "Nnrt factory name")
      ("model_path", po::value<std::string>(),
          "Path to the model")
      ("config_path", po::value<std::string>(),
          "YAML configuration file for the model")
      ("verbosity,v", po::value<int>(),
          "Verbosity level (0-3)")
      ("speed-only", po::bool_switch()->default_value(false),
          "Only run speed test; skip inference output test")
      ("speed-iterations", po::value<int>()->default_value(1000),
          "Number of measured speed-test inference iterations")
      ("speed-warmup", po::value<int>()->default_value(10),
          "Number of warmup inference iterations before speed test")
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

  if (arg_map.find("verbosity") != arg_map.end()) {
    STEPIT_SET_VERBOSITY(static_cast<VerbosityLevel>(arg_map["verbosity"].as<int>()));
  }

  const bool speed_only      = arg_map["speed-only"].as<bool>();
  const int speed_iterations = arg_map["speed-iterations"].as<int>();
  const int speed_warmup     = arg_map["speed-warmup"].as<int>();
  if (speed_iterations <= 0) {
    fmt::print(std::cerr, "{} Invalid speed-iterations '{}'. Expected a positive integer.\n", kErrorPrefix,
               speed_iterations);
    return -1;
  }
  if (speed_warmup < 0) {
    fmt::print(std::cerr, "{} Invalid speed-warmup '{}'. Expected a non-negative integer.\n", kErrorPrefix,
               speed_warmup);
    return -1;
  }

  PluginManager plugin_manager(plugin_args);

  auto factory      = arg_map["factory"].as<std::string>();
  const auto path   = arg_map["model_path"].as<std::string>();
  const auto config = (arg_map.find("config_path") != arg_map.end())
                          ? yml::loadFile(arg_map["config_path"].as<std::string>())
                          : yml::Node();
  if (startsWith(factory, "nnrt@")) {
    factory = factory.substr(std::strlen("nnrt@"));
  } else if (factory.find('@') != std::string::npos) {
    fmt::print(std::cerr, "{} Invalid factory name '{}'. Expected a factory name of nnrt.\n", kErrorPrefix, factory);
    return -1;
  }

  auto model = Nnrt::make(factory, path, config);
  model->printInfo();
  model->clearState();

  if (not speed_only) {
    displayFormattedBanner(60, nullptr, "Inference test");
    for (std::size_t step{1}; step <= 3; ++step) {
      fmt::print("Step {}:\n", step);
      // Set inputs
      std::vector<std::vector<float>> f32_inputs(model->getNumInputs());
      std::vector<std::vector<int32_t>> i32_inputs(model->getNumInputs());
      std::vector<std::vector<int64_t>> i64_inputs(model->getNumInputs());
      std::vector<std::vector<std::uint8_t>> bool_inputs(model->getNumInputs());
      for (std::size_t i{}; i < model->getNumInputs(); ++i) {
        if (not model->isInputRecurrent(i)) {
          const auto dtype = model->getInputDtype(i);
          const auto size  = model->getInputSize(i);
          if (dtype == DataType::kFloat32) {
            f32_inputs[i].assign(size, 0.01F * static_cast<float>(step));
            model->setInput(i, f32_inputs[i].data());
          } else if (dtype == DataType::kInt32) {
            i32_inputs[i].assign(size, static_cast<int32_t>(step));
            model->setInput(i, i32_inputs[i].data());
          } else if (dtype == DataType::kInt64) {
            i64_inputs[i].assign(size, static_cast<int64_t>(step));
            model->setInput(i, i64_inputs[i].data());
          } else if (dtype == DataType::kBool) {
            bool_inputs[i].assign(size, static_cast<std::uint8_t>(step % 2 == 1));
            model->setInput(i, static_cast<const void *>(bool_inputs[i].data()));
          }
        }
      }

      // Run inference
      model->runInference();

      // Print outputs
      for (std::size_t i{}; i < model->getNumOutputs(); ++i) {
        const auto size = static_cast<Eigen::Index>(model->getOutputSize(i));
        switch (model->getOutputDtype(i)) {
          case DataType::kUndefined:
            STEPIT_THROW("NNRT output '{}' has undefined data type.", model->getOutputName(i));
          case DataType::kFloat32: {
            using Array = Eigen::Array<float, Eigen::Dynamic, 1>;
            Eigen::Map<const Array> output(model->getOutput<float>(i), size);
            if (not model->isOutputRecurrent(i)) {
              std::cout << fmt::format("Output '{}':", model->getOutputName(i)) << output.transpose() << std::endl;
            }
            break;
          }
          case DataType::kInt32: {
            using Array = Eigen::Array<std::int32_t, Eigen::Dynamic, 1>;
            Eigen::Map<const Array> output(model->getOutput<std::int32_t>(i), size);
            if (not model->isOutputRecurrent(i)) {
              std::cout << fmt::format("Output '{}':", model->getOutputName(i)) << output.transpose() << std::endl;
            }
            break;
          }
          case DataType::kInt64: {
            using Array = Eigen::Array<std::int64_t, Eigen::Dynamic, 1>;
            Eigen::Map<const Array> output(model->getOutput<std::int64_t>(i), size);
            if (not model->isOutputRecurrent(i)) {
              std::cout << fmt::format("Output '{}':", model->getOutputName(i)) << output.transpose() << std::endl;
            }
            break;
          }
          case DataType::kBool: {
            using Array = Eigen::Array<bool, Eigen::Dynamic, 1>;
            Eigen::Map<const Array> output(model->getOutput<bool>(i), size);
            if (not model->isOutputRecurrent(i)) {
              std::cout << fmt::format("Output '{}':", model->getOutputName(i)) << output.transpose() << std::endl;
            }
            break;
          }
        }
      }
    }
  }

  std::vector<const void *> inputs(model->getNumInputs());
  std::vector<std::vector<float>> f32_inputs(model->getNumInputs());
  std::vector<std::vector<int32_t>> i32_inputs(model->getNumInputs());
  std::vector<std::vector<int64_t>> i64_inputs(model->getNumInputs());
  std::vector<std::vector<std::uint8_t>> bool_inputs(model->getNumInputs());
  for (std::size_t i{}; i < model->getNumInputs(); ++i) {
    if (not model->isInputRecurrent(i)) {
      const auto dtype = model->getInputDtype(i);
      const auto size  = model->getInputSize(i);
      if (dtype == DataType::kFloat32) {
        f32_inputs[i].assign(size, 0.0F);
        inputs[i] = f32_inputs[i].data();
      } else if (dtype == DataType::kInt32) {
        i32_inputs[i].assign(size, 0);
        inputs[i] = i32_inputs[i].data();
      } else if (dtype == DataType::kInt64) {
        i64_inputs[i].assign(size, 0);
        inputs[i] = i64_inputs[i].data();
      } else if (dtype == DataType::kBool) {
        bool_inputs[i].assign(size, 0);
        inputs[i] = bool_inputs[i].data();
      }
      model->setInput(i, inputs[i]);
    }
  }

  model->clearState();
  model->warmup(speed_warmup);
  model->clearState();

  displayFormattedBanner(60, nullptr, "Speed test");
  auto start_time = std::chrono::steady_clock::now();
  for (int step{}; step < speed_iterations; ++step) {
    for (std::size_t i{}; i < model->getNumInputs(); ++i) {
      if (not model->isInputRecurrent(i)) model->setInput(i, inputs[i]);
    }
    model->runInference();
    for (std::size_t i{}; i < model->getNumOutputs(); ++i) model->getOutput(i);
  }
  const auto elapsed      = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
  const double average_us = elapsed * 1e6 / static_cast<double>(speed_iterations);
  const double average_ms = average_us / 1e3;
  const double throughput = static_cast<double>(speed_iterations) / elapsed;

  fmt::print("Warmup iterations: {}\n", speed_warmup);
  fmt::print("Measured iterations: {}\n", speed_iterations);
  fmt::print("Total time: {:.3f} ms\n", elapsed * 1e3);
  fmt::print("Average latency: {:.3f} us ({:.6f} ms)\n", average_us, average_ms);
  fmt::print("Throughput: {:.3f} inference/s\n", throughput);
  return 0;
}

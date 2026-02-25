#include <fstream>
#include <iostream>
#include <string>

#include <onnxruntime_cxx_api.h>
#include <yaml-cpp/yaml.h>

#include <stepit/utils.h>

namespace fs = boost::filesystem;

int main(int argc, char **argv) {
  if (argc != 2 and argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <model.onnx> [output.yaml]" << std::endl;
    return 1;
  }

  try {
    const fs::path model_path = fs::canonical(argv[1]);
    fs::path output_path;
    if (argc == 3) {
      output_path = argv[2];
    } else {
      output_path = model_path.parent_path() / (model_path.stem().string() + "_metadata.yml");
    }

    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "onnx_metadata2yaml");
    Ort::SessionOptions opts;
    opts.SetInterOpNumThreads(1);
    opts.SetIntraOpNumThreads(1);
    Ort::Session session(env, model_path.c_str(), opts);

    Ort::AllocatorWithDefaultOptions allocator;
    auto metadata = session.GetModelMetadata();

    YAML::Node yaml;
    for (const auto &key : metadata.GetCustomMetadataMapKeysAllocated(allocator)) {
      const char *key_str   = key.get();
      const auto value      = metadata.LookupCustomMetadataMapAllocated(key_str, allocator);
      std::string value_str = value ? value.get() : "null";

      try {
        if (value_str.find(',') != std::string::npos) {
          value_str = "[" + value_str + "]";
        }
        yaml[key_str] = YAML::Load(value_str);
      } catch (const std::exception &) {
        yaml[key_str] = value.get();
      }
    }

    YAML::Emitter emitter;
    emitter.SetSeqFormat(YAML::Block);
    emitter << yaml;
    if (not emitter.good()) {
      throw std::runtime_error("Failed to emit YAML from ONNX metadata.");
    }

    std::ofstream out(output_path.string());
    if (not out.is_open()) {
      throw std::runtime_error("Failed to open output file: " + output_path.string());
    }
    out << emitter.c_str() << '\n';

    std::cout << "Saved metadata to: " << output_path << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "ERROR: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}

#include <iostream>
#include <vector>

#include <stepit/plugin.h>
#include <stepit/pyutils/npz_reader.h>

using namespace stepit;

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <npz_file>" << std::endl;
    return 1;
  }

  try {
    NpzReader npz(argv[1]);

    std::size_t max_name_len       = 4;
    std::size_t max_shape_len      = 5;
    std::vector<std::string> names = npz.keys();
    std::vector<std::string> shape_strs;
    std::vector<std::string> dtype_strs;

    for (const auto &name : names) {
      max_name_len = std::max(max_name_len, name.length());
      shape_strs.push_back(fmt::format("({})", fmt::join(npz[name].shape, ", ")));
      max_shape_len = std::max(max_shape_len, shape_strs.back().length());
      dtype_strs.push_back(npz[name].dtype);
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
    std::cerr << kRed << "ERROR: " << kClear << e.what() << std::endl;
    return 1;
  }

  return 0;
}

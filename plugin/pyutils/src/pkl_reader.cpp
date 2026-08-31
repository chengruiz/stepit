#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include "stepit/logging.h"

#include <stepit/pyutils/ndarray_utils.h>
#include <stepit/pyutils/pkl_reader.h>
#include <stepit/pyutils/python_runtime.h>

namespace stepit {
namespace field {
PklReader::PklReader(const std::string &path) {
  fs::path resolved_path(path);
  if (resolved_path.is_relative()) resolved_path = fs::absolute(resolved_path);
  if (not fs::exists(resolved_path)) {
    auto pkl_path = resolved_path;
    pkl_path.replace_extension(".pkl");
    if (fs::exists(pkl_path)) {
      resolved_path = std::move(pkl_path);
    } else {
      auto pickle_path = resolved_path;
      pickle_path.replace_extension(".pickle");
      if (fs::exists(pickle_path)) resolved_path = std::move(pickle_path);
    }
  }
  STEPIT_ASSERT(fs::exists(resolved_path), "File '{}' does not exist.", path);

  ensurePythonInterpreter();
  pybind11::gil_scoped_acquire gil;

  auto builtins = pybind11::module_::import("builtins");
  auto pickle   = pybind11::module_::import("pickle");
  auto numpy    = pybind11::module_::import("numpy");
  auto file     = builtins.attr("open")(fs::canonical(resolved_path).string(), "rb");
  auto data     = pickle.attr("load")(file);
  file.attr("close")();
  auto data_type = pybind11::str(pybind11::type::of(data)).cast<std::string>();

  STEPIT_ASSERT(pybind11::isinstance<pybind11::dict>(data), "Expected '{}' to contain a pickled dict, but got '{}'.",
                path, data_type);
  auto dict = data.cast<pybind11::dict>();

  for (const auto &item : dict) {
    if (not pybind11::isinstance<pybind11::str>(item.first)) continue;

    try {
      auto py_arr = numpy.attr("ascontiguousarray")(numpy.attr("asarray")(item.second)).cast<pybind11::array>();
      insert(item.first.cast<std::string>(), toNdArray(py_arr));
    } catch (const std::exception &error) {
      STEPIT_DBUGNT("Error processing item in pickle file: {}", error.what());
      continue;
    }
  }
}

STEPIT_REGISTER_DATALOADER(pkl, kDefPriority, DataLoader::make<PklReader>);
}  // namespace field
}  // namespace stepit

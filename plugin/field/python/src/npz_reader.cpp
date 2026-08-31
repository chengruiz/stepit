#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <stepit/pyutils/ndarray_utils.h>
#include <stepit/pyutils/npz_reader.h>
#include <stepit/pyutils/python_runtime.h>

namespace stepit {
namespace field {
NpzReader::NpzReader(const std::string &path) {
  fs::path resolved_path(path);
  if (resolved_path.is_relative()) resolved_path = fs::absolute(resolved_path);
  if (not fs::exists(resolved_path)) {
    resolved_path.replace_extension(".npz");
    STEPIT_ASSERT(fs::exists(resolved_path), "File '{}' does not exist.", path);
  }

  ensurePythonInterpreter();
  pybind11::gil_scoped_acquire gil;

  auto numpy = pybind11::module_::import("numpy");
  auto data  = numpy.attr("load")(fs::canonical(resolved_path).string(), pybind11::arg("allow_pickle") = false);
  auto files = data.attr("files").cast<std::vector<std::string>>();
  for (const auto &file : files) {
    auto py_arr = numpy.attr("ascontiguousarray")(data[file.c_str()]).cast<pybind11::array>();
    insert(file, toNdArray(py_arr));
  }
  data.attr("close")();
}

STEPIT_REGISTER_DATALOADER(npz, kDefPriority, DataLoader::make<NpzReader>);
}  // namespace field
}  // namespace stepit

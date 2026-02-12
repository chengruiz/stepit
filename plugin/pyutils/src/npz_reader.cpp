#include <memory>

#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <stepit/pyutils/npz_reader.h>

namespace stepit {
NdArray::~NdArray() {
  if (ptr != nullptr) delete[] ptr;
}

NdArray::NdArray(NdArray &&other) noexcept
    : ptr(other.ptr),
      shape(std::move(other.shape)),
      nbytes(other.nbytes),
      dtype(std::move(other.dtype)),
      itemsize(other.itemsize) {
  other.ptr      = nullptr;
  other.nbytes   = 0;
  other.itemsize = 0;
}

NdArray &NdArray::operator=(NdArray &&other) noexcept {
  if (this != &other) {
    if (ptr != nullptr) delete[] ptr;
    ptr            = other.ptr;
    shape          = std::move(other.shape);
    nbytes         = other.nbytes;
    dtype          = std::move(other.dtype);
    itemsize       = other.itemsize;
    other.ptr      = nullptr;
    other.nbytes   = 0;
    other.itemsize = 0;
  }
  return *this;
}

NpzReader::NpzReader(const std::string &path) { readFile(path); }

void NpzReader::readFile(const std::string &path) {
  keys_.clear();
  values_.clear();

  std::unique_ptr<pybind11::scoped_interpreter> guard;
  if (not Py_IsInitialized()) {
    guard = std::make_unique<pybind11::scoped_interpreter>();
  }
  auto numpy = pybind11::module_::import("numpy");
  auto data  = numpy.attr("load")(path);
  auto files = data.attr("files").cast<std::vector<std::string>>();
  for (const auto &file : files) {
    auto py_arr = data[file.c_str()].cast<pybind11::array>();
    auto py_buf = py_arr.request();

    NdArray array;
    array.shape.assign(py_buf.shape.begin(), py_buf.shape.end());
    array.dtype    = pybind11::str(py_arr.dtype()).cast<std::string>();
    array.nbytes   = py_buf.size * py_buf.itemsize;
    array.itemsize = py_buf.itemsize;
    array.ptr      = new char[array.nbytes];
    std::memcpy(array.ptr, py_buf.ptr, array.nbytes);

    keys_.push_back(file);
    values_.push_back(std::move(array));
  }
}

bool NpzReader::hasKey(const std::string &key) const {
  return std::find(keys_.begin(), keys_.end(), key) != keys_.end();
}

const NdArray &NpzReader::operator[](const std::string &key) const {
  auto it = std::find(keys_.begin(), keys_.end(), key);
  STEPIT_ASSERT(it != keys_.end(), "Key '{}' not found.", key);
  return values_[std::distance(keys_.begin(), it)];
}
}  // namespace stepit

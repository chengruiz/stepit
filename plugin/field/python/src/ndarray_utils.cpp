#include <stepit/pyutils/ndarray_utils.h>

namespace stepit {
namespace field {
NdArray toNdArray(const pybind11::array &py_arr) {
  const auto dtype = pybind11::str(py_arr.dtype()).cast<std::string>();
  STEPIT_ASSERT((py_arr.flags() & pybind11::array::c_style) != 0, "Expected a C-contiguous numpy array, but got '{}'.",
                dtype);
  STEPIT_ASSERT(not py_arr.dtype().attr("hasobject").cast<bool>(),
                "Unsupported numpy dtype '{}' because it contains Python objects.", dtype);

  auto py_buf = py_arr.request();

  NdArray array;
  array.shape.assign(py_buf.shape.begin(), py_buf.shape.end());
  array.dtype    = dtype;
  array.nbytes   = py_buf.size * py_buf.itemsize;
  array.itemsize = py_buf.itemsize;

  if (array.nbytes > 0) {
    array.ptr = new char[array.nbytes];
    std::memcpy(array.ptr, py_buf.ptr, array.nbytes);
  }
  return array;
}
}  // namespace field
}  // namespace stepit

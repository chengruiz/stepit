#ifndef STEPIT_PYUTILS_NDARRAY_UTILS_H_
#define STEPIT_PYUTILS_NDARRAY_UTILS_H_

#include <pybind11/numpy.h>

#include <stepit/field/data_loader.h>

namespace stepit {
namespace field {
NdArray toNdArray(const pybind11::array &py_arr);
}  // namespace field
}  // namespace stepit

#endif  // STEPIT_PYUTILS_NDARRAY_UTILS_H_

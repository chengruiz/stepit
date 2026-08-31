stepit_declare_plugin(NAME nnrt_tensorrt DEPENDS
    nnrt_base
)
if (POLICY CMP0146)
  cmake_policy(PUSH)
  cmake_policy(SET CMP0146 OLD)
endif ()

find_package(CUDA QUIET)

if (POLICY CMP0146)
  cmake_policy(POP)
endif ()
if (NOT CUDA_FOUND)
  stepit_plugin_mark_unbuildable("Missing CUDA.")
  return()
endif ()

get_library_directory(nvinfer TENSORRT_LIB_DIR)
if (NOT TENSORRT_LIB_DIR)
  stepit_plugin_mark_unbuildable("Missing TensorRT.")
  return()
endif ()

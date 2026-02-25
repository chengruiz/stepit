cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_nnrt_torchjit)
set(CMAKE_CXX_STANDARD 17)

find_package(Torch QUIET)
if (NOT Torch_FOUND)
  message(STATUS "Skipped plugin '${PROJECT_NAME}': Missing libtorch.")
  return()
endif ()

set(TORCH_PLUGIN_CXX_FLAGS "")
if (TORCH_CXX_FLAGS)
  separate_arguments(TORCH_PLUGIN_CXX_FLAGS NATIVE_COMMAND "${TORCH_CXX_FLAGS}")
endif ()

stepit_add_plugin(${PROJECT_NAME}
    SOURCES
      src/torchjit.cpp
    INCLUDES
      include
    LINK_LIBS
      ${TORCH_LIBRARIES}
    FLAGS
      ${TORCH_PLUGIN_CXX_FLAGS}
    DEPENDS
      nnrt_base
)

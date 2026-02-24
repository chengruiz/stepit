cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_pyutils)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG)
if (NOT pybind11_FOUND)
  init_submodule(extern/pybind11 WORKING_DIRECTORY ${STEPIT_HOME_DIRECTORY})
  add_subdirectory(${STEPIT_HOME_DIRECTORY}/extern/pybind11 pybind11)
endif()

stepit_add_plugin(${PROJECT_NAME}
    SOURCES
      src/npz_reader.cpp
    ENTRY
      src/entry.cpp
    INCLUDES
      include
    FLAGS
      -Wall -Wextra -Wpedantic -Wno-unused-parameter
    LINK_LIBS
      pybind11::embed
)

stepit_add_executable(npz_info app/npz_info.cpp)
target_link_libraries(npz_info PRIVATE ${PROJECT_NAME})

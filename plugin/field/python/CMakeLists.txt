cmake_minimum_required(VERSION 3.23)
project(stepit_plugin_pyutils)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
init_submodule(extern/pybind11 WORKING_DIRECTORY ${STEPIT_HOME_DIRECTORY})
add_subdirectory(${STEPIT_HOME_DIRECTORY}/extern/pybind11 pybind11)

stepit_add_plugin(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE
      src/ndarray_utils.cpp
      src/npz_reader.cpp
      src/pkl_reader.cpp
      src/python_runtime.cpp
    PUBLIC
      FILE_SET HEADERS
      BASE_DIRS include
      FILES
        include/stepit/pyutils/ndarray_utils.h
        include/stepit/pyutils/npz_reader.h
        include/stepit/pyutils/pkl_reader.h
        include/stepit/pyutils/python_runtime.h
)
target_link_libraries(${PROJECT_NAME} PUBLIC
    stepit_core
    stepit_plugin_field_base
    pybind11::embed
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Wno-unused-parameter)
install(TARGETS ${PROJECT_NAME}
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
set(entry_target ${PROJECT_NAME}_entry)
stepit_add_entry(${entry_target})
target_sources(${entry_target}
    PRIVATE
      src/entry.cpp
)
target_link_libraries(${entry_target} PRIVATE ${PROJECT_NAME})
install(TARGETS ${entry_target}
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
)

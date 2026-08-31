cmake_minimum_required(VERSION 3.23)
project(stepit_plugin_nnrt_tensorrt)

if (POLICY CMP0146)
  cmake_policy(PUSH)
  cmake_policy(SET CMP0146 OLD)
endif ()

find_package(CUDA REQUIRED)

if (POLICY CMP0146)
  cmake_policy(POP)
endif ()
get_library_directory(nvinfer TENSORRT_LIB_DIR REQUIRED)

stepit_add_plugin(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE
      src/tensorrt.cpp
    PUBLIC
      FILE_SET HEADERS
      BASE_DIRS include
      FILES
        include/stepit/nnrt/tensorrt.h
)
target_include_directories(${PROJECT_NAME} PUBLIC ${CUDA_INCLUDE_DIRS})
target_link_directories(${PROJECT_NAME} BEFORE PUBLIC ${TENSORRT_LIB_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC
    stepit_core
    stepit_plugin_nnrt_base
    ${CUDA_LIBRARIES}
    nvinfer
    nvonnxparser
)
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "${CMAKE_INSTALL_RPATH}:${TENSORRT_LIB_DIR}")
install(TARGETS ${PROJECT_NAME}
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

set(TENSORRT_INCLUDE_DIR ${TENSORRT_LIB_DIR}/../include)
if (EXISTS "${TENSORRT_INCLUDE_DIR}")
  target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${TENSORRT_INCLUDE_DIR})
endif ()

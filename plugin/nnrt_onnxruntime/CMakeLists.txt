cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_nnrt_onnxruntime)

if (NOT CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  message(STATUS "Skipped plugin '${PROJECT_NAME}': Requires x86_64 architecture, got '${CMAKE_SYSTEM_PROCESSOR}'.")
  return()
endif ()

get_library_directory(onnxruntime ONNXRUNTIME_LIB_DIR)
if (ONNXRUNTIME_LIB_DIR)
  message(STATUS "Found onnxruntime at ${ONNXRUNTIME_LIB_DIR}")
  get_filename_component(ONNXRUNTIME_DIR "${ONNXRUNTIME_LIB_DIR}" DIRECTORY)
else ()
  set(ONNXRUNTIME_TAG 1.21.0)
  set(NN_RUNTIME_DIR "${STEPIT_HOME_DIRECTORY}/extern/nn_runtime")
  set(ONNXRUNTIME_FULLNAME "onnxruntime-linux-x64-${ONNXRUNTIME_TAG}")
  set(ONNXRUNTIME_DIR "${NN_RUNTIME_DIR}/${ONNXRUNTIME_FULLNAME}")

  # Download onnxruntime if not already downloaded
  if (NOT EXISTS ${ONNXRUNTIME_DIR})
    file(MAKE_DIRECTORY ${NN_RUNTIME_DIR})

    set(ONNXRUNTIME_RELEASE_URL "https://github.com/microsoft/onnxruntime/releases/download")
    set(ONNXRUNTIME_FILENAME "${ONNXRUNTIME_FULLNAME}.tgz")
    set(ONNXRUNTIME_DOWNLOAD_URL "${ONNXRUNTIME_RELEASE_URL}/v${ONNXRUNTIME_TAG}/${ONNXRUNTIME_FILENAME}")
    set(ONNXRUNTIME_DOWNLOAD_PATH "${NN_RUNTIME_DIR}/${ONNXRUNTIME_FILENAME}")

    message(STATUS "Downloading onnxruntime from ${ONNXRUNTIME_DOWNLOAD_URL}")
    file(DOWNLOAD ${ONNXRUNTIME_DOWNLOAD_URL} ${ONNXRUNTIME_DOWNLOAD_PATH})

    # Extract the archive
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E tar xzf ${ONNXRUNTIME_DOWNLOAD_PATH}
        WORKING_DIRECTORY ${NN_RUNTIME_DIR}
    )

    # Delete the downloaded archive
    file(REMOVE ${ONNXRUNTIME_DOWNLOAD_PATH})
  endif ()
endif ()

set(ONNXRUNTIME_INCLUDE_DIR ${ONNXRUNTIME_DIR}/include)
if (EXISTS "${ONNXRUNTIME_INCLUDE_DIR}/onnxruntime")
  set(ONNXRUNTIME_INCLUDE_DIR ${ONNXRUNTIME_INCLUDE_DIR}/onnxruntime)
endif ()

stepit_add_plugin(${PROJECT_NAME}
    SOURCES
      src/onnxruntime.cpp
    INCLUDES
      include
      ${ONNXRUNTIME_INCLUDE_DIR}
    LINK_DIRS
      ${ONNXRUNTIME_DIR}/lib
    LINK_LIBS
      onnxruntime
    DEPENDS
      nnrt_base
)

stepit_add_executable(onnx_metadata2yaml app/onnx_metadata2yaml.cpp)
target_link_libraries(onnx_metadata2yaml PRIVATE ${PROJECT_NAME})

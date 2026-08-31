cmake_minimum_required(VERSION 3.23)
project(stepit_plugin_nnrt_onnxruntime)
set(CMAKE_CXX_STANDARD 17)

string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" ONNXRUNTIME_SYSTEM_PROCESSOR)
if (ONNXRUNTIME_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(ONNXRUNTIME_ARCHIVE_ARCH "x64")
elseif (ONNXRUNTIME_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(ONNXRUNTIME_ARCHIVE_ARCH "aarch64")
else ()
  message(FATAL_ERROR "Unsupported architecture '${CMAKE_SYSTEM_PROCESSOR}' for ${PROJECT_NAME}.")
endif ()

get_library_directory(onnxruntime ONNXRUNTIME_LIB_DIR)
if (ONNXRUNTIME_LIB_DIR)
  message(STATUS "Found onnxruntime at ${ONNXRUNTIME_LIB_DIR}")
  get_filename_component(ONNXRUNTIME_DIR "${ONNXRUNTIME_LIB_DIR}" DIRECTORY)
else ()
  set(ONNXRUNTIME_TAG 1.24.3)
  set(NN_RUNTIME_DIR "${STEPIT_HOME_DIRECTORY}/extern/nn_runtime")
  set(ONNXRUNTIME_FULLNAME "onnxruntime-linux-${ONNXRUNTIME_ARCHIVE_ARCH}-${ONNXRUNTIME_TAG}")
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

stepit_add_plugin(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE
      src/onnxruntime.cpp
    PUBLIC
      FILE_SET HEADERS
      BASE_DIRS include
      FILES
        include/stepit/nnrt/onnxruntime.h
)
target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${ONNXRUNTIME_INCLUDE_DIR}>)
target_link_directories(${PROJECT_NAME} BEFORE PUBLIC $<BUILD_INTERFACE:${ONNXRUNTIME_DIR}/lib>)
target_link_libraries(${PROJECT_NAME} PUBLIC
    stepit_core
    stepit_plugin_nnrt_base
    onnxruntime
)
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "${CMAKE_INSTALL_RPATH}:${ONNXRUNTIME_DIR}/lib")
install(TARGETS ${PROJECT_NAME}
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

add_executable(onnx_metadata2yaml app/onnx_metadata2yaml.cpp)
target_link_libraries(onnx_metadata2yaml PRIVATE ${PROJECT_NAME})
install(TARGETS onnx_metadata2yaml
    RUNTIME DESTINATION ${STEPIT_BIN_DESTINATION}
)

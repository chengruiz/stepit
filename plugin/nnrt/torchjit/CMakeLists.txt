cmake_minimum_required(VERSION 3.23)
project(stepit_plugin_nnrt_torchjit)
set(CMAKE_CXX_STANDARD 17)

find_package(Torch REQUIRED)

set(TORCH_PLUGIN_CXX_FLAGS "")
if (TORCH_CXX_FLAGS)
  separate_arguments(TORCH_PLUGIN_CXX_FLAGS NATIVE_COMMAND "${TORCH_CXX_FLAGS}")
endif ()

stepit_add_plugin(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE
      src/torchjit.cpp
    PUBLIC
      FILE_SET HEADERS
      BASE_DIRS include
      FILES
        include/stepit/nnrt/torchjit.h
)
target_link_libraries(${PROJECT_NAME} PUBLIC
    stepit_core
    stepit_plugin_nnrt_base
    ${TORCH_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${TORCH_PLUGIN_CXX_FLAGS})
install(TARGETS ${PROJECT_NAME}
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

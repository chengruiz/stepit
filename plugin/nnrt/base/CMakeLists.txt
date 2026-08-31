cmake_minimum_required(VERSION 3.23)
project(stepit_plugin_nnrt_base)

stepit_add_plugin(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE
      src/nnrt.cpp
    PUBLIC
      FILE_SET HEADERS
      BASE_DIRS include
      FILES
        include/stepit/nnrt/nnrt.h
)
target_link_libraries(${PROJECT_NAME} PUBLIC stepit_core)
install(TARGETS ${PROJECT_NAME}
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

add_executable(nnrt_test app/nnrt_test.cpp)
target_link_libraries(nnrt_test PRIVATE ${PROJECT_NAME} Boost::program_options)
install(TARGETS nnrt_test
    RUNTIME DESTINATION ${STEPIT_BIN_DESTINATION}
)

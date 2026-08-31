cmake_minimum_required(VERSION 3.23)
project(stepit_plugin_joystick_base)

stepit_add_plugin(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE
      src/joystick.cpp src/keymap.cpp
    PUBLIC
      FILE_SET HEADERS
      BASE_DIRS include
      FILES
        include/stepit/joystick/joystick.h
        include/stepit/joystick/keymap.h
        include/stepit/joystick/state.h
)
target_link_libraries(${PROJECT_NAME} PUBLIC stepit_core)
install(TARGETS ${PROJECT_NAME}
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

add_executable(joystick_test app/joystick_test.cpp)
target_link_libraries(joystick_test
    PRIVATE
      ${PROJECT_NAME}
      Boost::program_options
)
install(TARGETS joystick_test
    RUNTIME DESTINATION ${STEPIT_BIN_DESTINATION}
)

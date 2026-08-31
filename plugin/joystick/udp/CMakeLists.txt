cmake_minimum_required(VERSION 3.23)
project(stepit_plugin_joystick_udp)

set(UDP_GAMEPAD_BUILD_EXAMPLE OFF)
init_submodule(extern/joystick_sdk/udp-gamepad WORKING_DIRECTORY ${STEPIT_HOME_DIRECTORY})
add_subdirectory(${STEPIT_HOME_DIRECTORY}/extern/joystick_sdk/udp-gamepad udp-gamepad)
target_sources(udp_gamepad
    PUBLIC
      FILE_SET HEADERS
      BASE_DIRS ${STEPIT_HOME_DIRECTORY}/extern/joystick_sdk/udp-gamepad/include
      FILES
        ${STEPIT_HOME_DIRECTORY}/extern/joystick_sdk/udp-gamepad/include/udp_gamepad/gamepad.h
        ${STEPIT_HOME_DIRECTORY}/extern/joystick_sdk/udp-gamepad/include/udp_gamepad/gamepad_keys.h
        ${STEPIT_HOME_DIRECTORY}/extern/joystick_sdk/udp-gamepad/include/udp_gamepad/retroid_gamepad.h
        ${STEPIT_HOME_DIRECTORY}/extern/joystick_sdk/udp-gamepad/include/udp_gamepad/skydroid_gamepad.h
        ${STEPIT_HOME_DIRECTORY}/extern/joystick_sdk/udp-gamepad/include/udp_gamepad/udp_receiver.h
)
set_target_properties(udp_gamepad PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES
      "$<BUILD_INTERFACE:${STEPIT_HOME_DIRECTORY}/extern/joystick_sdk/udp-gamepad/include>;$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

stepit_add_plugin(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE
      src/joystick_udp.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC
    stepit_core
    stepit_plugin_joystick_base
    udp_gamepad
)
install(TARGETS ${PROJECT_NAME}
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
)

install(TARGETS udp_gamepad
    EXPORT stepitTargets
    RUNTIME DESTINATION ${STEPIT_BIN_DESTINATION}
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

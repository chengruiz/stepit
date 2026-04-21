cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_joystick_base)

stepit_add_plugin(${PROJECT_NAME}
    SOURCES src/joystick.cpp src/keymap.cpp
    INCLUDES include
)

stepit_add_executable(joystick_test app/joystick_test.cpp)
target_link_libraries(joystick_test PRIVATE ${PROJECT_NAME})

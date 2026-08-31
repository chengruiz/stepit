cmake_minimum_required(VERSION 3.23)
project(stepit_plugin_modular_policy_locomotion)

stepit_add_plugin(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE
      src/cmd_posture_source.cpp
      src/cmd_vel_source.cpp
      src/heightmap_source.cpp
      src/odometry_source.cpp
      src/proprioceptor.cpp
    PUBLIC
      FILE_SET HEADERS
      BASE_DIRS include
      FILES
        include/stepit/modular_policy/locomotion/cmd_posture_source.h
        include/stepit/modular_policy/locomotion/cmd_vel_source.h
        include/stepit/modular_policy/locomotion/heightmap_source.h
        include/stepit/modular_policy/locomotion/odometry_source.h
        include/stepit/modular_policy/locomotion/proprioceptor.h
)
target_link_libraries(${PROJECT_NAME} PUBLIC
    stepit_core
    stepit_plugin_modular_policy_base
    stepit_plugin_joystick_base
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Wno-unused-parameter)
install(TARGETS ${PROJECT_NAME}
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

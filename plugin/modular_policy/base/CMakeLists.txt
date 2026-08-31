cmake_minimum_required(VERSION 3.23)
project(stepit_plugin_modular_policy_base)

stepit_add_plugin(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE
      src/action_source.cpp
      src/actuator.cpp
      src/field_ops.cpp
      src/joint_reordering.cpp
      src/module.cpp
      src/modular_policy.cpp
      src/subscriber_action.cpp
      src/time_source.cpp
    PUBLIC
      FILE_SET HEADERS
      BASE_DIRS include
      FILES
        include/stepit/modular_policy/action_source.h
        include/stepit/modular_policy/actuator.h
        include/stepit/modular_policy/field_ops.h
        include/stepit/modular_policy/joint_reordering.h
        include/stepit/modular_policy/module.h
        include/stepit/modular_policy/modular_policy.h
        include/stepit/modular_policy/subscriber_action.h
        include/stepit/modular_policy/time_source.h
)
target_link_libraries(${PROJECT_NAME} PUBLIC
    stepit_core
    stepit_plugin_field_base
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Wno-unused-parameter)
install(TARGETS ${PROJECT_NAME}
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

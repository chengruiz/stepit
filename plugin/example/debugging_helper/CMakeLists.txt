cmake_minimum_required(VERSION 3.23)
project(stepit_plugin_debugging_helper)

stepit_add_plugin(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE
      src/dummy_control.cpp
      src/dummy_robot.cpp
      src/exit_spin.cpp
    PUBLIC
      FILE_SET HEADERS
      BASE_DIRS include
      FILES
        include/stepit/debugging/dummy_control.h
        include/stepit/debugging/dummy_robot.h
        include/stepit/debugging/exit_spin.h
)
target_link_libraries(${PROJECT_NAME} PUBLIC stepit_core)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Wno-unused-parameter)
install(TARGETS ${PROJECT_NAME}
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
set(entry_target ${PROJECT_NAME}_entry)
stepit_add_entry(${entry_target})
target_sources(${entry_target}
    PRIVATE
      src/entry.cpp
)
target_link_libraries(${entry_target} PRIVATE ${PROJECT_NAME})
install(TARGETS ${entry_target}
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
)

cmake_minimum_required(VERSION 3.23)

set(VALID_ROBOTS "aliengo;go1;b1")
if (NOT STEPIT_UNITREE_ROBOT IN_LIST VALID_ROBOTS)
  message(FATAL_ERROR "Unsupported robot '${STEPIT_UNITREE_ROBOT}'")
endif ()

project(stepit_plugin_robot_unitree_${STEPIT_UNITREE_ROBOT})

get_library_directory(lcm LCM_DIR REQUIRED)

if (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(ARCH amd64)
elseif (CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(ARCH arm64)
else ()
  message(FATAL_ERROR "Unsupported architecture '${CMAKE_SYSTEM_PROCESSOR}' for ${PROJECT_NAME}.")
endif ()

init_submodule(extern/robot_sdk/unitree_${STEPIT_UNITREE_ROBOT}_sdk WORKING_DIRECTORY ${STEPIT_HOME_DIRECTORY})
set(UNITREE_LEGGED_SDK_HOME ${STEPIT_HOME_DIRECTORY}/extern/robot_sdk/unitree_${STEPIT_UNITREE_ROBOT}_sdk)

stepit_add_plugin(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/src/unitree.cpp
    PUBLIC
      FILE_SET HEADERS
      BASE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
      FILES ${CMAKE_CURRENT_LIST_DIR}/include/stepit/robot/unitree.h
)
target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${UNITREE_LEGGED_SDK_HOME}/include>)
target_link_directories(${PROJECT_NAME} BEFORE PUBLIC $<BUILD_INTERFACE:${UNITREE_LEGGED_SDK_HOME}/lib/cpp/${ARCH}>)
target_link_libraries(${PROJECT_NAME} PUBLIC
    stepit_core
    unitree_legged_sdk
    lcm
)
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "${CMAKE_INSTALL_RPATH}:${UNITREE_LEGGED_SDK_HOME}/lib/cpp/${ARCH}")
install(TARGETS ${PROJECT_NAME}
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

string(TOUPPER "UNITREE_${STEPIT_UNITREE_ROBOT}" ROBOT_MACRO)
target_compile_definitions(${PROJECT_NAME} PUBLIC BOOST_BIND_GLOBAL_PLACEHOLDERS STEPIT_UNITREE_ROBOT=${ROBOT_MACRO})

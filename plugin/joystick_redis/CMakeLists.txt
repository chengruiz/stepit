cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_joystick_redis)

find_package(PkgConfig QUIET)
if (NOT PkgConfig_FOUND)
  message(STATUS "Skipped plugin '${PROJECT_NAME}': Missing PkgConfig.")
  return()
endif ()

pkg_check_modules(HIREDIS QUIET hiredis)
if (NOT HIREDIS_FOUND)
  message(STATUS "Skipped plugin '${PROJECT_NAME}': Missing hiredis.")
  return()
endif ()

find_package(nlohmann_json QUIET)
if (NOT nlohmann_json_FOUND)
  message(STATUS "Skipped plugin '${PROJECT_NAME}': Missing nlohmann_json.")
  return()
endif ()

stepit_add_plugin(${PROJECT_NAME}
    SOURCES
      src/joystick_redis.cpp
    DEPENDS
      joystick_base
      redis_base
)

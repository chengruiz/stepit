cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_redis_base)

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
      src/redis_client.cpp
    INCLUDES
      include
      ${HIREDIS_INCLUDE_DIRS}
    LINK_LIBS
      ${HIREDIS_LIBRARIES}
      nlohmann_json::nlohmann_json
    LINK_DIRS
      ${HIREDIS_LIBRARY_DIRS}
    FLAGS
      -Wall -Wextra -Wpedantic -Wno-unused-parameter
      ${HIREDIS_CFLAGS_OTHER}
)

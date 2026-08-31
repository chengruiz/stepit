cmake_minimum_required(VERSION 3.23)
project(stepit_plugin_redis_base)

find_package(PkgConfig REQUIRED)
pkg_check_modules(HIREDIS REQUIRED hiredis)
find_package(nlohmann_json REQUIRED)

stepit_add_plugin(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE
      src/redis_client.cpp
    PUBLIC
      FILE_SET HEADERS
      BASE_DIRS include
      FILES
        include/stepit/redis/redis_client.h
)
target_include_directories(${PROJECT_NAME} PUBLIC ${HIREDIS_INCLUDE_DIRS})
target_link_directories(${PROJECT_NAME} BEFORE PUBLIC ${HIREDIS_LIBRARY_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC
    stepit_core
    ${HIREDIS_LIBRARIES}
    nlohmann_json::nlohmann_json
)
target_compile_options(${PROJECT_NAME} PRIVATE
    -Wall -Wextra -Wpedantic -Wno-unused-parameter
    ${HIREDIS_CFLAGS_OTHER}
)
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "${CMAKE_INSTALL_RPATH}:${HIREDIS_LIBRARY_DIRS}")
install(TARGETS ${PROJECT_NAME}
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

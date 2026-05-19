cmake_minimum_required(VERSION 3.23)
project(stepit)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

if (NOT DEFINED CMAKE_LIBRARY_OUTPUT_DIRECTORY AND
    NOT DEFINED CMAKE_RUNTIME_OUTPUT_DIRECTORY AND
    NOT DEFINED CMAKE_ARCHIVE_OUTPUT_DIRECTORY)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
endif ()

set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
if (CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
else ()
  message(
      WARNING
      "Building in '${CMAKE_BUILD_TYPE}' mode. "
      "Add '-DCMAKE_BUILD_TYPE=Release' for better performance."
  )
endif ()
if (NOT DEFINED STEPIT_LIB_DESTINATION)
  set(STEPIT_LIB_DESTINATION "${CMAKE_INSTALL_LIBDIR}")
endif ()
if (NOT DEFINED STEPIT_BIN_DESTINATION)
  set(STEPIT_BIN_DESTINATION "${CMAKE_INSTALL_BINDIR}")
endif ()
file(RELATIVE_PATH INSTALL_RPATH
    ${CMAKE_INSTALL_PREFIX}/${STEPIT_BIN_DESTINATION}
    ${CMAKE_INSTALL_PREFIX}/${STEPIT_LIB_DESTINATION}
)
set(CMAKE_INSTALL_RPATH "$ORIGIN/${INSTALL_RPATH}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)

set(STEPIT_HOME_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}")
include(${CMAKE_CURRENT_LIST_DIR}/cmake/stepit_utils.cmake)

find_package(Boost REQUIRED COMPONENTS filesystem program_options)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
init_submodule(extern/llu WORKING_DIRECTORY ${STEPIT_HOME_DIRECTORY})
add_subdirectory(extern/llu)

add_library(stepit_core SHARED
    src/stepit/agent.cpp
    src/stepit/communication.cpp
    src/stepit/control_input.cpp
    src/stepit/logging.cpp
    src/stepit/plugin.cpp
    src/stepit/publisher.cpp
    src/stepit/robot.cpp
    src/stepit/spin.cpp
    src/stepit/utils.cpp
)
target_compile_definitions(stepit_core PRIVATE STEPIT_CONFIG_DIR="${STEPIT_HOME_DIRECTORY}/config")
target_compile_definitions(stepit_core PUBLIC FMT_HEADER_ONLY=1)
target_compile_options(stepit_core PRIVATE -Wall -Wextra -Wpedantic -Wno-unused-parameter)
target_sources(stepit_core
    PUBLIC
      FILE_SET HEADERS
      BASE_DIRS include
      FILES
        include/stepit/agent.h
        include/stepit/communication.h
        include/stepit/control_input.h
        include/stepit/logging.h
        include/stepit/plugin.h
        include/stepit/policy.h
        include/stepit/publisher.h
        include/stepit/registry.h
        include/stepit/robot.h
        include/stepit/spin.h
        include/stepit/utils.h
)
target_link_libraries(stepit_core PUBLIC
    Boost::filesystem
    Eigen3::Eigen
    Threads::Threads
    atomic
    llu::llu
    yaml-cpp
)
target_link_libraries(stepit_core PRIVATE
    dl
)

# Check for gettid() function
include(CheckCSourceCompiles)
check_c_source_compiles("#include <unistd.h>
int main() { (void) gettid(); return 0; }
" HAVE_GETTID)
if (NOT HAVE_GETTID)
  target_compile_definitions(stepit_core PUBLIC STEPIT_FIX_GETTID)
endif ()

add_executable(stepit src/stepit_app/main.cpp)
target_link_libraries(stepit PRIVATE stepit_core Boost::program_options)
install(TARGETS stepit
    RUNTIME DESTINATION ${STEPIT_BIN_DESTINATION}
)

add_executable(rotation_transform src/stepit_app/rotation_transform.cpp)
target_link_libraries(rotation_transform PRIVATE stepit_core Boost::program_options)
install(TARGETS rotation_transform
    RUNTIME DESTINATION ${STEPIT_BIN_DESTINATION}
)

stepit_add_plugin(stepit_plugin_builtin)
target_sources(stepit_plugin_builtin
    PRIVATE
      src/stepit/builtin.cpp
)
target_link_libraries(stepit_plugin_builtin PUBLIC stepit_core)
install(TARGETS stepit_plugin_builtin
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
)

# Dynamically discover plugins from manifest files
set(STEPIT_DISCOVERED_PLUGINS "")
set(STEPIT_DISCOVERED_PLUGIN_DIRS "")
# Add the default plugin search directory
list(APPEND STEPIT_PLUGIN_DIRS "${CMAKE_CURRENT_LIST_DIR}/plugin")
foreach (plugin_search_dir ${STEPIT_PLUGIN_DIRS})
  if (NOT IS_ABSOLUTE ${plugin_search_dir})
    set(plugin_search_dir "$ENV{PWD}/${plugin_search_dir}")
  endif ()
  if (NOT IS_DIRECTORY ${plugin_search_dir})
    message(WARNING "Plugin directory '${plugin_search_dir}' does not exist.")
    continue()
  endif ()

  # Search for plugins in the directory
  file(GLOB discovered_plugin_dirs RELATIVE ${plugin_search_dir} ${plugin_search_dir}/*)
  foreach (plugin_entry ${discovered_plugin_dirs})
    set(plugin_dir "${plugin_search_dir}/${plugin_entry}")
    get_filename_component(plugin_dir "${plugin_dir}" REALPATH)
    if (
        IS_DIRECTORY ${plugin_dir}
        AND EXISTS ${plugin_dir}/stepit_plugin_manifest.cmake
        AND NOT ${plugin_dir} STREQUAL ${CMAKE_CURRENT_LIST_DIR}
    )
      if (plugin_dir IN_LIST STEPIT_DISCOVERED_PLUGIN_DIRS)
        continue()
      endif ()
      list(APPEND STEPIT_DISCOVERED_PLUGIN_DIRS "${plugin_dir}")

      stepit_load_plugin_manifest(${plugin_dir} manifest_id)
      stepit_get_plugin_manifest_property(${manifest_id} NAME plugin_name)
      if (plugin_name IN_LIST STEPIT_DISCOVERED_PLUGINS)
        stepit_get_plugin_property(${plugin_name} DIR existing_plugin_dir)
        message(
            FATAL_ERROR
            "Duplicated plugin '${plugin_name}' declared in "
            "'${existing_plugin_dir}' and '${plugin_dir}'."
        )
      endif ()
      stepit_commit_plugin_manifest(${manifest_id})
      list(APPEND STEPIT_DISCOVERED_PLUGINS ${plugin_name})
    endif ()
  endforeach ()
endforeach ()
list(SORT STEPIT_DISCOVERED_PLUGINS)
message(STATUS "Discovered stepit plugins: ${STEPIT_DISCOVERED_PLUGINS}")

if (NOT DEFINED STEPIT_BLACKLIST_PLUGINS)
  set(STEPIT_BLACKLIST_PLUGINS "")
endif ()
set(STEPIT_ENABLED_PLUGINS "${STEPIT_DISCOVERED_PLUGINS}")
foreach (blacklisted_plugin ${STEPIT_BLACKLIST_PLUGINS})
  list(REMOVE_ITEM STEPIT_ENABLED_PLUGINS ${blacklisted_plugin})
endforeach ()
message(STATUS "Blacklisted plugins: ${STEPIT_BLACKLIST_PLUGINS}")

if (NOT DEFINED STEPIT_WHITELIST_PLUGINS)
  set(STEPIT_WHITELIST_PLUGINS "${STEPIT_ENABLED_PLUGINS}")
endif ()
message(STATUS "Requested stepit plugins: ${STEPIT_WHITELIST_PLUGINS}")

# Resolve the requested plugin graph before building any plugin targets.
set_property(GLOBAL PROPERTY STEPIT_RESOLVED_PLUGINS "")
set_property(GLOBAL PROPERTY STEPIT_BUILDABLE_PLUGIN_ORDER "")
set_property(GLOBAL PROPERTY STEPIT_BUILT_PLUGINS "")

foreach (plugin ${STEPIT_WHITELIST_PLUGINS})
  if (NOT ${plugin} IN_LIST STEPIT_DISCOVERED_PLUGINS)
    message(FATAL_ERROR "Plugin '${plugin}' is not supported.")
  endif ()
  stepit_resolve_plugin(${plugin} "")
endforeach ()

get_property(STEPIT_RESOLVED_PLUGINS GLOBAL PROPERTY STEPIT_RESOLVED_PLUGINS)
get_property(STEPIT_BUILDABLE_PLUGIN_ORDER GLOBAL PROPERTY STEPIT_BUILDABLE_PLUGIN_ORDER)
message(STATUS "Resolved stepit plugin closure: ${STEPIT_RESOLVED_PLUGINS}")
message(STATUS "Buildable stepit plugins: ${STEPIT_BUILDABLE_PLUGIN_ORDER}")

# Build the buildable plugins in dependency order.
foreach (plugin ${STEPIT_BUILDABLE_PLUGIN_ORDER})
  stepit_get_plugin_property(${plugin} DEPENDS plugin_dependencies)
  set(skip_plugin FALSE)
  foreach (dependency ${plugin_dependencies})
    stepit_get_plugin_property(${dependency} STATUS dependency_status)
    if (NOT dependency_status STREQUAL "built")
      stepit_mark_plugin_skipped_due_to_dependency(${plugin} ${dependency})
      set(skip_plugin TRUE)
      break()
    endif ()
  endforeach ()
  if (skip_plugin)
    continue()
  endif ()

  stepit_get_plugin_property(${plugin} DIR plugin_dir)
  add_subdirectory(${plugin_dir} plugins/${plugin})

  if (TARGET stepit_plugin_${plugin})
    stepit_set_plugin_property(${plugin} STATUS "built")
    set_property(GLOBAL APPEND PROPERTY STEPIT_BUILT_PLUGINS ${plugin})
  else ()
    stepit_set_plugin_property(${plugin} STATUS "skipped")
    stepit_set_plugin_property(
        ${plugin}
        REASON
        "Plugin CMakeLists.txt did not create target 'stepit_plugin_${plugin}'. Check manifest requirements."
    )
    stepit_set_plugin_property(${plugin} CHAIN ${plugin})
  endif ()
endforeach ()

stepit_print_plugin_report(${STEPIT_RESOLVED_PLUGINS})

install(TARGETS stepit_core
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

get_property(STEPIT_BUILT_PLUGINS GLOBAL PROPERTY STEPIT_BUILT_PLUGINS)
if (NOT STEPIT_BUILT_PLUGINS)
  set(STEPIT_BUILT_PLUGINS "")
endif ()

set(stepit_dependencies_file "${CMAKE_CURRENT_BINARY_DIR}/stepitDependencies.cmake")
file(WRITE "${stepit_dependencies_file}" "set(STEPIT_BUILT_PLUGINS \"${STEPIT_BUILT_PLUGINS}\")\n")
foreach (plugin ${STEPIT_BUILT_PLUGINS})
  stepit_get_plugin_property(${plugin} DEPENDS plugin_dependencies)
  file(APPEND "${stepit_dependencies_file}" "set(STEPIT_PLUGIN_DEPENDENCIES_${plugin} \"${plugin_dependencies}\")\n")
endforeach ()

install(EXPORT stepitTargets
    FILE stepitTargets.cmake
    NAMESPACE stepit::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/stepit
)
configure_package_config_file(
    "${STEPIT_HOME_DIRECTORY}/cmake/stepitConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/stepitConfig.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/stepit"
)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/stepitConfigVersion.cmake"
    VERSION 0.0.0
    COMPATIBILITY SameMajorVersion
)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/stepitConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/stepitConfigVersion.cmake"
    "${stepit_dependencies_file}"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/stepit"
)

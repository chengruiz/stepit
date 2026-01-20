cmake_minimum_required(VERSION 3.16)
project(stepit)

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
  message(WARNING "Building in '${CMAKE_BUILD_TYPE}' mode, which may affect real-time performance.")
endif ()
if (NOT DEFINED STEPIT_LIB_DESTINATION)
  set(STEPIT_LIB_DESTINATION "lib")
endif ()
if (NOT DEFINED STEPIT_BIN_DESTINATION)
  set(STEPIT_BIN_DESTINATION "bin")
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

stepit_add_library(stepit_core SHARED
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
target_compile_definitions(stepit_core PRIVATE STEPIT_CONFIG_DIR=\"${STEPIT_HOME_DIRECTORY}/config\")
target_compile_options(stepit_core PUBLIC -fPIC PRIVATE -Wall -Wextra -Wpedantic -Wno-unused-parameter)
target_include_directories(stepit_core PUBLIC ${STEPIT_HOME_DIRECTORY}/include)
target_link_libraries(stepit_core PUBLIC
    atomic
    Boost::filesystem
    Boost::program_options
    Eigen3::Eigen
    dl
    llu
    Threads::Threads
    yaml-cpp
)

# Check for gettid() function
include(CheckCSourceCompiles)
check_c_source_compiles("#include <unistd.h>
int main() { (void) gettid(); return 0; }
" HAVE_GETTID)
if (NOT HAVE_GETTID)
  target_compile_definitions(stepit_core PUBLIC STEPIT_FIX_GETTID)
endif ()

stepit_add_executable(stepit src/stepit_app/main.cpp)
stepit_add_executable(rotation_transform src/stepit_app/rotation_transform.cpp)
stepit_add_plugin(stepit_plugin_builtin SOURCES src/stepit/builtin.cpp)

# Dynamically discover plugins
set(STEPIT_ALL_PLUGINS "")
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
  file(GLOB discovered_plugins RELATIVE ${plugin_search_dir} ${plugin_search_dir}/*)
  foreach (plugin ${discovered_plugins})
    set(plugin_dir "${plugin_search_dir}/${plugin}")
    get_filename_component(plugin_dir "${plugin_dir}" REALPATH)
    if (
        IS_DIRECTORY ${plugin_dir}
        AND EXISTS ${plugin_dir}/CMakeLists.txt
        AND NOT EXISTS ${plugin_dir}/STEPIT_IGNORE
        AND NOT ${plugin_dir} STREQUAL ${CMAKE_CURRENT_LIST_DIR}
    )
      if (${plugin} IN_LIST STEPIT_ALL_PLUGINS)
        message(WARNING "Duplicated plugin '${plugin}'.")
        continue()
      endif ()
      set(STEPIT_PLUGIN_${plugin}_DIR "${plugin_dir}")
      list(APPEND STEPIT_ALL_PLUGINS ${plugin})
    endif ()
  endforeach ()
endforeach ()
list(SORT STEPIT_ALL_PLUGINS)
message(STATUS "Discovered stepit plugins: ${STEPIT_ALL_PLUGINS}")

if (NOT DEFINED STEPIT_BLACKLIST_PLUGINS)
  set(STEPIT_BLACKLIST_PLUGINS "")
endif ()
foreach (blacklisted_plugin ${STEPIT_BLACKLIST_PLUGINS})
  list(REMOVE_ITEM STEPIT_ALL_PLUGINS ${blacklisted_plugin})
endforeach ()
message(STATUS "Blacklisted plugins: ${STEPIT_BLACKLIST_PLUGINS}")

if (NOT DEFINED STEPIT_WHITELIST_PLUGINS)
  set(STEPIT_WHITELIST_PLUGINS "${STEPIT_ALL_PLUGINS}")
endif ()

# Build the whitelisted plugins
while (STEPIT_WHITELIST_PLUGINS)
  list(POP_FRONT STEPIT_WHITELIST_PLUGINS plugin)
  if (NOT ${plugin} IN_LIST STEPIT_ALL_PLUGINS)
    message(FATAL_ERROR "Plugin '${plugin}' is not supported.")
  endif ()
  if (TARGET stepit_plugin_${plugin})
    continue()
  endif ()

  add_subdirectory(${STEPIT_PLUGIN_${plugin}_DIR} plugins/${plugin})
  # If the plugin is successfully built, add it to the list of built plugins
  if (TARGET stepit_plugin_${plugin})
    set_property(GLOBAL APPEND PROPERTY STEPIT_BUILT_PLUGINS ${plugin})
    # Resolve plugin dependencies
    get_property(plugin_dependencies TARGET stepit_plugin_${plugin} PROPERTY STEPIT_PLUGIN_DEPENDENCIES)
    foreach (dependency ${plugin_dependencies})
      if (NOT dependency IN_LIST STEPIT_BUILT_PLUGINS AND
          NOT dependency IN_LIST STEPIT_WHITELIST_PLUGINS)
        list(PREPEND STEPIT_WHITELIST_PLUGINS ${dependency})
      endif ()
    endforeach ()
  endif ()
endwhile ()

get_property(STEPIT_LIBRARIES GLOBAL PROPERTY STEPIT_LIBRARIES)
get_property(STEPIT_EXECUTABLES GLOBAL PROPERTY STEPIT_EXECUTABLES)
install(TARGETS ${STEPIT_LIBRARIES} ${STEPIT_EXECUTABLES}
    RUNTIME DESTINATION ${STEPIT_BIN_DESTINATION}
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
)

function(stepit_add_library library_name)
  add_library(${library_name} ${ARGN})
  set_property(GLOBAL APPEND PROPERTY STEPIT_LIBRARIES ${library_name})
endfunction()

function(stepit_add_plugin plugin_name)
  # assert plugin_name starts with "stepit_plugin_"
  if (NOT plugin_name MATCHES "^stepit_plugin_")
    message(FATAL_ERROR "Plugin name must start with 'stepit_plugin_'")
  endif ()

  # argument structure
  set(options "")
  set(oneValueArgs "")
  set(multiValueArgs SOURCES ENTRY INCLUDES LINK_LIBS LINK_DIRS FLAGS DEPENDS)

  cmake_parse_arguments(PARSE_ARGV 1 ARG
      "${options}"
      "${oneValueArgs}"
      "${multiValueArgs}"
  )
  if (ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "Unparsed arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif ()

  stepit_add_library(${plugin_name} SHARED ${ARG_SOURCES})
  target_include_directories(${plugin_name} PUBLIC ${ARG_INCLUDES})
  target_link_libraries(${plugin_name} PUBLIC stepit_core ${ARG_LINK_LIBS})
  target_link_directories(${plugin_name} BEFORE PUBLIC ${ARG_LINK_DIRS})
  target_compile_options(${plugin_name} PUBLIC ${ARG_FLAGS})
  set_target_properties(${plugin_name} PROPERTIES
      INSTALL_RPATH "${CMAKE_INSTALL_RPATH}:${ARG_LINK_DIRS}"
  )
  set_property(TARGET ${plugin_name} PROPERTY STEPIT_PLUGIN_DEPENDENCIES "${ARG_DEPENDS}")

  foreach (dependency ${ARG_DEPENDS})
    add_dependencies(${plugin_name} stepit_plugin_${dependency})
    target_link_libraries(${plugin_name} PUBLIC stepit_plugin_${dependency})
  endforeach ()

  if (ARG_ENTRY)
    set(plugin_entry "${plugin_name}_entry")
    stepit_add_library(${plugin_entry} MODULE ${ARG_ENTRY})
    target_link_libraries(${plugin_entry} PUBLIC ${plugin_name})
  endif ()
  message(STATUS "Added plugin '${plugin_name}'")
endfunction()

function(stepit_add_executable executable_name)
  add_executable(${executable_name} ${ARGN})
  target_link_libraries(${executable_name} PRIVATE stepit_core)
  set_property(GLOBAL APPEND PROPERTY STEPIT_EXECUTABLES ${executable_name})
endfunction()

function(get_library_directory library_name output_var)
  set(options "REQUIRED;VERBOSE")
  set(oneValueArgs "")
  set(multiValueArgs "")

  cmake_parse_arguments(PARSE_ARGV 2 ARG
      "${options}"
      "${oneValueArgs}"
      "${multiValueArgs}"
  )
  if (ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "Unparsed arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif ()

  # First find the library in the LD_LIBRARY_PATH
  string(REPLACE ":" ";" LD_LIBRARY_PATH "$ENV{LD_LIBRARY_PATH}")
  find_library(${library_name}_PATH NAMES ${library_name} PATHS ${LD_LIBRARY_PATH} NO_DEFAULT_PATH)
  # If not found, try to find it in the system library paths
  if (NOT ${library_name}_PATH)
    find_library(${library_name}_PATH NAMES ${library_name})
  endif ()

  # If the library is found, set the output variable to the directory
  if (${library_name}_PATH)
    if (ARG_VERBOSE)
      message(STATUS "Found ${library_name} in ${${library_name}_PATH}")
    endif ()
    get_filename_component(library_dir ${${library_name}_PATH} DIRECTORY)
    set(${output_var} ${library_dir} PARENT_SCOPE)
  elseif (ARG_REQUIRED)
    message(FATAL_ERROR "Library ${library_name} not found.")
  endif ()
endfunction()

function(init_submodule submodule_name)
  find_package(Git REQUIRED)

  set(options "RECURSIVE")
  set(oneValueArgs "WORKING_DIRECTORY")
  set(multiValueArgs "")

  cmake_parse_arguments(PARSE_ARGV 1 ARG
      "${options}"
      "${oneValueArgs}"
      "${multiValueArgs}"
  )
  if (ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "Unparsed arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif ()
  if (NOT ARG_WORKING_DIRECTORY)
    set(ARG_WORKING_DIRECTORY ${CMAKE_HOME_DIRECTORY})
  endif ()

  # Check if the submodule is already initialized
  execute_process(
      COMMAND ${GIT_EXECUTABLE} submodule status ${submodule_name}
      WORKING_DIRECTORY ${ARG_WORKING_DIRECTORY}
      OUTPUT_VARIABLE status_output
      OUTPUT_STRIP_TRAILING_WHITESPACE
  )

  if (status_output MATCHES "^-.*")
    message(STATUS "Initializing submodule: ${submodule_name}")
    if (ARG_RECURSIVE)
      set(recursive_flag "--recursive")
    endif ()
    execute_process(
        COMMAND ${GIT_EXECUTABLE} submodule update --init ${recursive_flag} ${submodule_name}
        WORKING_DIRECTORY ${ARG_WORKING_DIRECTORY}
        RESULT_VARIABLE update_result
    )

    if (NOT update_result EQUAL "0")
      message(FATAL_ERROR "git submodule update failed for ${submodule_name}")
    endif ()
  endif ()
endfunction()

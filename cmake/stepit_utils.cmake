function(stepit_set_plugin_property plugin property_name)
  if (ARGC GREATER 2)
    set_property(GLOBAL PROPERTY "STEPIT_PLUGIN_${plugin}_${property_name}" "${ARGN}")
  else ()
    set_property(GLOBAL PROPERTY "STEPIT_PLUGIN_${plugin}_${property_name}" "")
  endif ()
endfunction()

function(stepit_get_plugin_property plugin property_name output_var)
  get_property(property_value GLOBAL PROPERTY "STEPIT_PLUGIN_${plugin}_${property_name}")
  if (NOT DEFINED property_value)
    set(property_value "")
  endif ()
  set(${output_var} "${property_value}" PARENT_SCOPE)
endfunction()

function(stepit_set_plugin_manifest_property manifest_id property_name)
  if (ARGC GREATER 2)
    set_property(GLOBAL PROPERTY "STEPIT_PLUGIN_MANIFEST_${manifest_id}_${property_name}" "${ARGN}")
  else ()
    set_property(GLOBAL PROPERTY "STEPIT_PLUGIN_MANIFEST_${manifest_id}_${property_name}" "")
  endif ()
endfunction()

function(stepit_get_plugin_manifest_property manifest_id property_name output_var)
  get_property(property_value GLOBAL PROPERTY "STEPIT_PLUGIN_MANIFEST_${manifest_id}_${property_name}")
  if (NOT DEFINED property_value)
    set(property_value "")
  endif ()
  set(${output_var} "${property_value}" PARENT_SCOPE)
endfunction()

function(stepit_append_unique_global_property property_name)
  get_property(property_values GLOBAL PROPERTY ${property_name})
  if (NOT property_values)
    set(property_values "")
  endif ()

  foreach (property_value ${ARGN})
    if (NOT property_value IN_LIST property_values)
      list(APPEND property_values ${property_value})
    endif ()
  endforeach ()

  set_property(GLOBAL PROPERTY ${property_name} "${property_values}")
endfunction()

function(stepit_add_plugin plugin_name)
  if (NOT plugin_name MATCHES "^stepit_plugin_")
    message(FATAL_ERROR "Plugin name must start with 'stepit_plugin_'")
  endif ()
  if (TARGET ${plugin_name})
    message(FATAL_ERROR "Plugin target '${plugin_name}' already exists.")
  endif ()
  add_library(${plugin_name} SHARED)
endfunction()

function(stepit_add_entry entry_name)
  if (NOT entry_name MATCHES "^stepit_plugin_.*_entry$")
    message(FATAL_ERROR "Entry name must start with 'stepit_plugin_' and end with '_entry'")
  endif ()
  if (TARGET ${entry_name})
    message(FATAL_ERROR "Entry target '${entry_name}' already exists.")
  endif ()
  add_library(${entry_name} MODULE)
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

function(stepit_plugin_mark_unbuildable reason)
  set(STEPIT_PLUGIN_BUILDABLE FALSE PARENT_SCOPE)
  set(STEPIT_PLUGIN_REASON "${reason}" PARENT_SCOPE)
endfunction()

function(stepit_declare_plugin)
  set(options "")
  set(oneValueArgs NAME)
  set(multiValueArgs DEPENDS)

  cmake_parse_arguments(PARSE_ARGV 0 ARG
      "${options}"
      "${oneValueArgs}"
      "${multiValueArgs}"
  )
  if (ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "Unparsed arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif ()
  if (NOT ARG_NAME)
    message(FATAL_ERROR "stepit_declare_plugin requires NAME.")
  endif ()

  set(STEPIT_PLUGIN_NAME "${ARG_NAME}" PARENT_SCOPE)
  set(STEPIT_PLUGIN_DEPENDS "${ARG_DEPENDS}" PARENT_SCOPE)
endfunction()

function(stepit_load_plugin_manifest plugin_dir output_var)
  string(MD5 manifest_id "${plugin_dir}")
  stepit_set_plugin_manifest_property(${manifest_id} DIR "${plugin_dir}")

  set(STEPIT_PLUGIN_MANIFEST_ID "${manifest_id}")
  # Isolate directory properties that package find scripts may mutate.
  add_subdirectory(
      "${STEPIT_HOME_DIRECTORY}/cmake/plugin_manifest_scope"
      "${CMAKE_CURRENT_BINARY_DIR}/plugin_manifest_scopes/${manifest_id}"
  )

  set(${output_var} "${manifest_id}" PARENT_SCOPE)
endfunction()

function(stepit_commit_plugin_manifest manifest_id)
  stepit_get_plugin_manifest_property(${manifest_id} NAME plugin_name)
  stepit_get_plugin_manifest_property(${manifest_id} DIR STEPIT_PLUGIN_DIR)
  stepit_get_plugin_manifest_property(${manifest_id} DEPENDS STEPIT_PLUGIN_DEPENDS)
  stepit_get_plugin_manifest_property(${manifest_id} BUILDABLE STEPIT_PLUGIN_BUILDABLE)
  stepit_get_plugin_manifest_property(${manifest_id} REASON STEPIT_PLUGIN_REASON)

  stepit_set_plugin_property(${plugin_name} DIR "${STEPIT_PLUGIN_DIR}")
  stepit_set_plugin_property(${plugin_name} DEPENDS ${STEPIT_PLUGIN_DEPENDS})
  stepit_set_plugin_property(${plugin_name} BUILDABLE ${STEPIT_PLUGIN_BUILDABLE})
  stepit_set_plugin_property(${plugin_name} REASON "${STEPIT_PLUGIN_REASON}")
endfunction()

function(stepit_resolve_plugin plugin dependency_stack)
  if (NOT plugin IN_LIST STEPIT_DISCOVERED_PLUGINS)
    message(FATAL_ERROR "Plugin '${plugin}' is not supported.")
  endif ()

  stepit_append_unique_global_property(STEPIT_RESOLVED_PLUGINS ${plugin})

  stepit_get_plugin_property(${plugin} STATUS current_status)
  if (current_status STREQUAL "buildable" OR
      current_status STREQUAL "built" OR
      current_status STREQUAL "skipped")
    return()
  endif ()

  if (plugin IN_LIST dependency_stack)
    list(APPEND dependency_stack ${plugin})
    string(REPLACE ";" " -> " cycle_text "${dependency_stack}")
    message(FATAL_ERROR "Detected plugin dependency cycle: ${cycle_text}")
  endif ()

  list(APPEND dependency_stack ${plugin})

  if (NOT plugin IN_LIST STEPIT_ENABLED_PLUGINS)
    stepit_set_plugin_property(${plugin} STATUS "skipped")
    stepit_set_plugin_property(${plugin} REASON "Plugin '${plugin}' is blacklisted.")
    stepit_set_plugin_property(${plugin} CHAIN ${plugin})
    return()
  endif ()

  stepit_get_plugin_property(${plugin} BUILDABLE plugin_buildable)
  stepit_get_plugin_property(${plugin} REASON plugin_reason)
  if (NOT plugin_buildable)
    stepit_set_plugin_property(${plugin} STATUS "skipped")
    stepit_set_plugin_property(${plugin} REASON "${plugin_reason}")
    stepit_set_plugin_property(${plugin} CHAIN ${plugin})
    return()
  endif ()

  stepit_get_plugin_property(${plugin} DEPENDS plugin_dependencies)
  foreach (dependency ${plugin_dependencies})
    if (NOT dependency IN_LIST STEPIT_DISCOVERED_PLUGINS)
      message(
          FATAL_ERROR
          "Plugin '${plugin}' declares unknown dependency '${dependency}'."
      )
    endif ()

    stepit_resolve_plugin(${dependency} "${dependency_stack}")
    stepit_get_plugin_property(${dependency} STATUS dependency_status)
    if (NOT dependency_status STREQUAL "buildable" AND
        NOT dependency_status STREQUAL "built")
      stepit_mark_plugin_skipped_due_to_dependency(${plugin} ${dependency})
      return()
    endif ()
  endforeach ()

  stepit_set_plugin_property(${plugin} STATUS "buildable")
  stepit_set_plugin_property(${plugin} REASON "")
  stepit_set_plugin_property(${plugin} CHAIN ${plugin})
  stepit_append_unique_global_property(STEPIT_BUILDABLE_PLUGIN_ORDER ${plugin})
endfunction()

function(stepit_mark_plugin_skipped_due_to_dependency plugin dependency)
  stepit_set_plugin_property(${plugin} STATUS "skipped")
  stepit_set_plugin_property(${plugin} REASON "Dependency plugin '${dependency}' is skipped.")
  stepit_set_plugin_property(${plugin} CHAIN ${plugin} ${dependency})
endfunction()

function(stepit_print_plugin_report)
  get_property(built_plugins GLOBAL PROPERTY STEPIT_BUILT_PLUGINS)
  if (NOT built_plugins)
    set(built_plugins "")
  endif ()
  set(skipped_plugins "")

  foreach (plugin ${ARGN})
    stepit_get_plugin_property(${plugin} STATUS plugin_status)
    if (plugin_status STREQUAL "skipped")
      list(APPEND skipped_plugins ${plugin})
    endif ()
  endforeach ()

  if (built_plugins)
    message(STATUS "Built stepit plugins in order:")
    foreach (plugin ${built_plugins})
      message(STATUS "  ${plugin}")
    endforeach ()
  else ()
    message(STATUS "Built stepit plugins in order: none")
  endif ()

  if (skipped_plugins)
    message(STATUS "Skipped stepit plugins:")
    foreach (plugin ${skipped_plugins})
      stepit_get_plugin_property(${plugin} REASON plugin_reason)
      stepit_get_plugin_property(${plugin} CHAIN plugin_chain)
      if (plugin_chain)
        string(REPLACE ";" " -> " plugin_chain_text "${plugin_chain}")
        message(STATUS "  ${plugin_chain_text}: ${plugin_reason}")
      else ()
        message(STATUS "  ${plugin}: ${plugin_reason}")
      endif ()
    endforeach ()
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

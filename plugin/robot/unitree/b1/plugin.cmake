stepit_declare_plugin(NAME robot_unitree_b1)

if (NOT CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|aarch64")
  stepit_plugin_mark_unbuildable(
      "Requires x86_64 or aarch64 architectures, got '${CMAKE_SYSTEM_PROCESSOR}'."
  )
  return()
endif ()

get_library_directory(lcm LCM_DIR)
if (NOT LCM_DIR)
  stepit_plugin_mark_unbuildable("Missing LCM.")
  return()
endif ()

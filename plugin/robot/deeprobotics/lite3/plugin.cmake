stepit_declare_plugin(NAME robot_deeprobotics_lite3)

if (NOT CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|aarch64")
  stepit_plugin_mark_unbuildable(
      "Requires x86_64 or aarch64 architectures, got '${CMAKE_SYSTEM_PROCESSOR}'."
  )
  return()
endif ()

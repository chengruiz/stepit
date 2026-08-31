stepit_declare_plugin(NAME pyutils DEPENDS
    field_base
)
find_package(Python3 QUIET COMPONENTS Interpreter Development)
if (NOT Python3_FOUND)
  stepit_plugin_mark_unbuildable("Missing Python3 interpreter or development files.")
  return()
endif ()

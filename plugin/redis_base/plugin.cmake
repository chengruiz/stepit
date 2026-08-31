stepit_declare_plugin(NAME redis_base)

find_package(PkgConfig QUIET)
if (NOT PkgConfig_FOUND)
  stepit_plugin_mark_unbuildable("Missing PkgConfig.")
  return()
endif ()

pkg_check_modules(HIREDIS QUIET hiredis)
if (NOT HIREDIS_FOUND)
  stepit_plugin_mark_unbuildable("Missing hiredis.")
  return()
endif ()

find_package(nlohmann_json QUIET)
if (NOT nlohmann_json_FOUND)
  stepit_plugin_mark_unbuildable("Missing nlohmann_json.")
  return()
endif ()

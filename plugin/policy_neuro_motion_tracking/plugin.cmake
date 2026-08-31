stepit_declare_plugin(NAME policy_neuro_motion_tracking DEPENDS
    policy_neuro
    pyutils
)
find_package(pinocchio QUIET)
if (NOT pinocchio_FOUND)
  stepit_plugin_mark_unbuildable("Missing pinocchio.")
  return()
endif ()

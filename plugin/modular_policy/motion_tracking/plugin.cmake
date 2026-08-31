stepit_declare_plugin(NAME modular_policy_motion_tracking DEPENDS
    modular_policy_base
    joystick_base
    pyutils
)
find_package(pinocchio QUIET)
if (NOT pinocchio_FOUND)
  stepit_plugin_mark_unbuildable("Missing pinocchio.")
  return()
endif ()

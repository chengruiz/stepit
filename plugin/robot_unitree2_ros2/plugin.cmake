stepit_declare_plugin(NAME robot_unitree2_ros2 DEPENDS
    robot_unitree2
    ros2_base
)
find_package(unitree_go QUIET)
if (NOT unitree_go_FOUND)
  stepit_plugin_mark_unbuildable("Missing unitree_go.")
  return()
endif ()

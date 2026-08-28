stepit_declare_plugin(NAME policy_neuro_ros DEPENDS
    policy_neuro
    ros_base
)
find_package(catkin QUIET COMPONENTS geometry_msgs grid_map_ros nav_msgs roscpp std_msgs tf2_ros topic_tools)
if (NOT catkin_FOUND)
  stepit_plugin_mark_unbuildable("Missing catkin.")
  return()
endif ()

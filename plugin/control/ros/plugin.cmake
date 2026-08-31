stepit_declare_plugin(NAME control_ros DEPENDS ros_base)

find_package(catkin QUIET COMPONENTS roscpp std_msgs)
if (NOT catkin_FOUND)
  stepit_plugin_mark_unbuildable("Missing catkin.")
endif ()

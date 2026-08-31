stepit_declare_plugin(NAME ros_base)

find_package(catkin QUIET COMPONENTS roscpp)
if (NOT catkin_FOUND)
  stepit_plugin_mark_unbuildable("Missing catkin.")
endif ()

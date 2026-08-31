stepit_declare_plugin(NAME ros_base DEPENDS
    joystick_base
)
find_package(catkin QUIET COMPONENTS roscpp sensor_msgs std_msgs)
if (NOT catkin_FOUND)
  stepit_plugin_mark_unbuildable("Missing catkin.")
  return()
endif ()

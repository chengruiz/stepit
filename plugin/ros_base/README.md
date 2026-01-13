# stepit_plugin_ros_base

StepIt plugin providing integration with ROS for communication, control, and input.
Tested with ROS Noetic (Ubuntu 20.04).

### Environment Variables

- `STEPIT_ROS_NODE_NAME` (string): the name of the stepit ROS node.
- `STEPIT_JOY_NAME` (string): the name of the joystick to use for `ros` joystick input.

### Provided Factories

- `stepit::ControlInput`:
  - `ros_msg`: controlling via ROS topic (`/control` of type `std_msgs/String`)
  - `ros_srv`: controlling via ROS service (`/control` of type `stepit_ros_msgs/Control`)
- `stepit::Publisher`:
  - `ros`: publishing diagnostic status, IMU, and joint states as ROS topics.
- `stepit::Spin`:
  - `ros`: using ROS event loop for spinning.
- `stepit::joystick::Joystick`:
  - `ros`: providing joystick input from ROS topic (`/joy` of type `sensor_msgs/Joy`).
  - `ros_xbox`: providing joystick input from ROS topic with Xbox keymap.

### Notes

- StepIt can be put in the `src` directory of your ROS workspace and built with `catkin build`. It cannot be built with `catkin_make` or `catkin_make_isolated` unless you manually add a `CATKIN_IGNORE` file under `package/ros2`.

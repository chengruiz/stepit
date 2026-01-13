# stepit_plugin_ros2_base

StepIt plugin providing integration with ROS2 for communication, control, and input.
Tested with ROS2 Humble (Ubuntu 22.04) and Jazzy (Ubuntu 24.04).

### Environment Variables

- `STEPIT_ROS2_NODE_NAME` (string): the name of the stepit ROS2 node.
- `STEPIT_ROS2_QOS_RELIABILITY` (string): the default QoS reliability policy for ROS2 topics.
- `STEPIT_ROS2_QOS_DURABILITY` (string): the default QoS durability policy for ROS2 topics.
- `STEPIT_ROS2_QOS_HISTORY` (string): the default QoS history policy for ROS2 topics.
- `STEPIT_JOY_NAME` (string): the name of the joystick to use for `ros2` joystick input.

### Provided Factories

- `stepit::ControlInput`: 
  - `ros2_msg`: controlling via ROS2 topic (`/control` of type `std_msgs/String`)
  - `ros2_srv`: controlling via ROS2 service (`/control` of type `stepit_ros2_msgs/Control`)
- `stepit::Publisher`:
  - `ros2`: publishing diagnostic status, IMU, and joint states as ROS2 topics.
- `stepit::Spin`:
  - `ros2`: using ROS2 event loop for spinning.
- `stepit::joystick::Joystick`:
  - `ros2`: providing joystick input from ROS2 topic (`/joy` of type `sensor_msgs/Joy`).
  - `ros2_xbox`: providing joystick input from ROS2 topic with Xbox keymap.

### Notes 

- To use `ros2_srv` control input when building with `cmake`, you need to add `install/lib` to `LD_LIBRARY_PATH`, e.g.

```shell
export LD_LIBRARY_PATH=install/lib:$LD_LIBRARY_PATH
```

- To build StepIt with `colcon`, you should add `stepit/plugin/ros2` to `--base-paths` in your `colcon` command.
You also need to skip the `stepit` package to avoid linkage error, e.g.

```shell
colcon build --base-paths src src/stepit/package/ros2 --packages-skip stepit
```

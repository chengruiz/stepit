# joystick_ros2

StepIt plugin providing ROS2 joystick input. It depends on `joystick_base` and `ros2_base`.

### Environment Variables

- `STEPIT_JOY_NAME` (string): joystick keymap name used by the `ros2` joystick input.

### Provided Factories

- `stepit::joystick::Joystick`:
  - `ros2`: receives `sensor_msgs/msg/Joy` messages from the `joy_topic` parameter (default `/joy`).
  - `ros2_xbox`: receives the same topic using the Xbox keymap.

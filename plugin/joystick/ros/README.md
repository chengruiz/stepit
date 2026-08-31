# joystick_ros

StepIt plugin providing joystick input from ROS topics.
Tested with ROS Noetic (Ubuntu 20.04).

### Environment Variables

- `STEPIT_JOY_NAME` (string): the joystick keymap to use for the `ros` joystick input.

### Provided Factories

- `stepit::joystick::Joystick`:
  - `ros`: receives `/joy` messages of type `sensor_msgs/Joy`.
  - `ros_xbox`: receives `/joy` messages with the Xbox keymap.

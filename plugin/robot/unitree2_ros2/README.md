# robot_unitree2_ros2

StepIt plugin providing integration with Unitree2 robots via [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) for control, state communication, and joystick input.
Tested with ROS2 Humble (Ubuntu 22.04).

Features:

- Command publisher for low-level control (`/lowcmd`, `u2ros2_msg::LowCmd`).
- State subscriber for low-level feedback (`/lowstate`, `u2ros2_msg::LowState`).
- Joystick input from ROS2 topic (`/wirelesscontroller`, `u2ros2_msg::WirelessController`).

### Provided Factories

- `stepit::RobotApi`: `b2_ros2`, `go2_ros2`
- `stepit::joystick::Joystick`: `unitree2_ros2`

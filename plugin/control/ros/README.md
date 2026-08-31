# control_ros

StepIt plugin providing control input through ROS topics and services.
Tested with ROS Noetic (Ubuntu 20.04).

### Provided Factories

- `stepit::ControlInput`:
  - `ros_msg`: controls through the `/control` topic (`std_msgs/String`).
  - `ros_srv`: controls through the `/control` service (`stepit_ros_msgs/Control`).

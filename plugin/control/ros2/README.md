# control_ros2

StepIt plugin providing ROS2 control inputs. It depends on `ros2_base` for the shared node.

### Provided Factories

- `stepit::ControlInput`:
  - `ros2_msg`: accepts control requests from the `control` topic (`std_msgs/msg/String`).
  - `ros2_srv`: accepts control requests through the `control` service (`stepit_ros2_msgs/srv/Control`).

### Notes

- When building directly with CMake, `ros2_srv` requires the generated service library to be discoverable, for example:

  ```shell
  export LD_LIBRARY_PATH=install/lib:$LD_LIBRARY_PATH
  ```

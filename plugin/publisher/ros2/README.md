# publisher_ros2

StepIt plugin publishing status, low-level robot state, and named arrays as ROS2 topics. It depends on `ros2_base`.

### Provided Factories

- `stepit::Publisher`:
  - `ros2`: publishes diagnostic status, IMU, joint states, and runtime-typed arrays.

### Array Topic Types

Named arrays preserve their runtime scalar type:

| StepIt data type | ROS2 message type |
| :--------------- | :---------------- |
| `float32` | `std_msgs/msg/Float32MultiArray` |
| `int32` | `std_msgs/msg/Int32MultiArray` |
| `int64` | `std_msgs/msg/Int64MultiArray` |
| `bool` | `std_msgs/msg/UInt8MultiArray` (0/1) |

A topic's message type is fixed by its first publication. Publishing the same topic later with a different type is rejected.

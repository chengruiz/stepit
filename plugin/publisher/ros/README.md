# publisher_ros

StepIt plugin publishing diagnostic status, low-level robot state, and runtime-typed arrays as ROS topics.
Tested with ROS Noetic (Ubuntu 20.04).

### Provided Factories

- `stepit::Publisher`: `ros`

### Array Topic Types

Named arrays preserve their runtime scalar type:

| StepIt data type | ROS message type                |
| :--------------- | :------------------------------ |
| `float32`        | `std_msgs/Float32MultiArray`    |
| `int32`          | `std_msgs/Int32MultiArray`      |
| `int64`          | `std_msgs/Int64MultiArray`      |
| `bool`           | `std_msgs/UInt8MultiArray` (0/1) |

A topic's message type is fixed by its first publication; publishing the same topic later with a different data type
is rejected.

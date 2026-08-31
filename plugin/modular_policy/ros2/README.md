# modular_policy_ros2

StepIt plugin for generic ROS2 array-field subscribers in the modular policy pipeline.

### Provided Factories

`stepit::modular_policy::Module`:

- `field_subscriber`: subscribes to configured array topics and provides named fields. Supported `dtype` values and
  message types are `float32` / `std_msgs/msg/Float32MultiArray`, `int32` / `std_msgs/msg/Int32MultiArray`, `int64` /
  `std_msgs/msg/Int64MultiArray`, and `bool` / `std_msgs/msg/UInt8MultiArray`.

Locomotion-specific ROS2 sources are provided by `modular_policy_locomotion_ros2`.

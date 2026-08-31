# modular_policy_ros

StepIt plugin for generic ROS array-field subscribers in the modular policy pipeline.

### Provided Factories

`stepit::modular_policy::Module`:

- `field_subscriber`: subscribes to configured array topics and provides named fields. Supported `dtype` values and
  message types are `float32` / `std_msgs/Float32MultiArray`, `int32` / `std_msgs/Int32MultiArray`, `int64` /
  `std_msgs/Int64MultiArray`, and `bool` / `std_msgs/UInt8MultiArray`.

Locomotion-specific ROS sources are provided by `modular_policy_locomotion_ros`.

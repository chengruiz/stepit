# ros2_base

StepIt plugin providing the shared ROS2 node, QoS helpers, and event loop.
Tested with ROS2 Humble (Ubuntu 22.04) and Jazzy (Ubuntu 24.04).

### Environment Variables

- `STEPIT_ROS2_NODE_NAME` (string): the name of the stepit ROS2 node.
- `STEPIT_ROS2_QOS_RELIABILITY` (string): the default QoS reliability policy for ROS2 topics.
- `STEPIT_ROS2_QOS_DURABILITY` (string): the default QoS durability policy for ROS2 topics.
- `STEPIT_ROS2_QOS_HISTORY` (string): the default QoS history policy for ROS2 topics.

### Provided Factories

- `stepit::Spin`:
  - `ros2`: using ROS2 event loop for spinning.

### Notes 

- To build StepIt with `colcon`, you should add `stepit/plugin/integration/ros2` to `--base-paths` in your `colcon`
  command.
You also need to skip the `stepit` package to avoid linkage error, e.g.

  ```shell
  colcon build --base-paths src src/stepit/package/ros2 --packages-skip stepit
  ```

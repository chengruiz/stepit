# ros_base

StepIt plugin providing the ROS node, common node-handle helpers, and ROS event loop.
Tested with ROS Noetic (Ubuntu 20.04).

### Environment Variables

- `STEPIT_ROS_NODE_NAME` (string): the name of the StepIt ROS node.

### Provided Factories

- `stepit::Spin`: `ros`, using the ROS event loop for spinning.

### Notes

- StepIt can be put in the `src` directory of a ROS workspace and built with `catkin build`. It cannot be built with
  `catkin_make` or `catkin_make_isolated` unless a `CATKIN_IGNORE` file is added under `package/ros2`.

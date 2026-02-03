# policy_neuro_ros2

StepIt plugin for ROS2-based field sources that subscribe ROS2 topics and feed data into the StepIt neuro policy.

### Provided Factories

`stepit::policy_neuro::Module`:

- `cmd_height_subscriber`: subscribes to a ROS2 topic of one of the following types and provides command height field (`cmd_height`):
    - `std_msgs/msg/Float32`,
    - `geometry_msgs/msg/Twist` (`linear.z` component),
    - `geometry_msgs/msg/TwistStamped` (`linear.z` component).
- `cmd_pitch_subscriber`: subscribes to a ROS2 topic of one of the following types and provides command pitch field (`cmd_pitch`):
    - `std_msgs/msg/Float32`,
    - `geometry_msgs/msg/Twist` (`angular.y` component),
    - `geometry_msgs/msg/TwistStamped` (`angular.y` component).
- `cmd_roll_subscriber`: subscribes to a ROS2 topic of one of the following types and provides command roll field (`cmd_roll`):
    - `std_msgs/msg/Float32`,
    - `geometry_msgs/msg/Twist` (`angular.x` component),
    - `geometry_msgs/msg/TwistStamped` (`angular.x` component).
- `cmd_vel_subscriber`: subscribes to a ROS2 topic of one of the following types and provides command velocity fields (`cmd_vel`) with `linear.x`, `linear.y`, and `angular.z` components:
    - `geometry_msgs/msg/Twist`,
    - `geometry_msgs/msg/TwistStamped`.
- `heightmap_subscriber`: subscribes to an elevation map topic of type `grid_map_msgs/msg/GridMap` and a pose topic of one of the following types, sample elevation and uncertainty values around the robot, and provide corresponding fields (`heightmap` / `heightmap_uncertainty`):
    - `geometry_msgs/msg/PoseStamped`,
    - `geometry_msgs/msg/PoseWithCovarianceStamped`,
    - `nav_msgs/msg/Odometry`.


### Joystick Key Bindings

- `LB` + `A`: enables velocity and posture command subcriptions, or disables them if already enabled.
- `LB` + `B`: enables heightmap and uncertainty subcriptions, or disables them if already enabled.


### Notes

- The `CmdVelSubscriber2` will be used as the preferred source for the `cmd_vel` field, as it has a higher priority than the `CmdHeightSource` in the `policy_neuro` plugin. To completely disable the subscription, specify `cmd_vel` in the `module` item of the policy configuration to use the `CmdHeightSource` class, which will prevent it from searching for other sources. Do the same for `cmd_height`, `cmd_pitch`, and `cmd_roll` if needed.

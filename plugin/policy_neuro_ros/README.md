# policy_neuro_ros

StepIt plugin for ROS-based field sources that subscribe ROS topics and feed data into the StepIt neuro policy.

### Provided Factories

`stepit::policy_neuro::Module`:
  - `cmd_height_subscriber`: subscribes to a ROS topic of one of the following types and provides command height field (`cmd_height`):
    - `std_msgs/Float32`,
    - `geometry_msgs/Twist` (`linear.z` component),
    - `geometry_msgs/TwistStamped` (`linear.z` component).
  - `cmd_pitch_subscriber`: subscribes to a ROS topic of one of the following types and provides command pitch field (`cmd_pitch`):
    - `std_msgs/Float32`,
    - `geometry_msgs/Twist` (`angular.y` component),
    - `geometry_msgs/TwistStamped` (`angular.y` component).
  - `cmd_roll_subscriber`: subscribes to a ROS topic of one of the following types and provides command roll field (`cmd_roll`):
    - `std_msgs/Float32`,
    - `geometry_msgs/Twist` (`angular.x` component),
    - `geometry_msgs/TwistStamped` (`angular.x` component).
  - `cmd_vel_subscriber`: subscribes to a ROS topic of one of the following types and provides command velocity fields (`cmd_vel`) with `linear.x`, `linear.y`, and `angular.z` components:
    - `geometry_msgs/Twist`,
    - `geometry_msgs/TwistStamped`.
  - `heightmap_subscriber`: subscribes to an elevation map topic of type `grid_map_msgs/GridMap` and a pose topic of one of the following types, sample elevation and uncertainty values around the robot, and provide corresponding fields (`heightmap` / `heightmap_uncertainty`):
    - `geometry_msgs/PoseStamped`,
    - `geometry_msgs/PoseWithCovarianceStamped`,
    - `nav_msgs/Odometry`.


### Joystick Key Bindings

- `LB` + `A`: enables velocity and posture command subcriptions, or disables them if already enabled.
- `LB` + `B`: enables heightmap and uncertainty subcriptions, or disables them if already enabled.


### Notes

- The `CmdVelSubscriber` will be used as the preferred source for the `cmd_vel` field, as it has a higher priority than the `CmdHeightSource` in the `policy_neuro` plugin. To completely disable the subscription, specify `cmd_vel` in the `module` item of the policy configuration to use the `CmdHeightSource` class, which will prevent it from searching for other sources. Do the same for `cmd_height`, `cmd_pitch`, and `cmd_roll` if needed.
